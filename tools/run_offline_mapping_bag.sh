#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${1:-}"
SCENE_DIR="${2:-}"
PLAY_RATE="${3:-0.5}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
LOOP_SEARCH_RADIUS="${LOOP_SEARCH_RADIUS:-10.0}"
SUPERLIO_SAVE_TIMEOUT_SECONDS="${SUPERLIO_SAVE_TIMEOUT_SECONDS:-660}"
OFFLINE_PROCESS_TIMEOUT_SECONDS="${OFFLINE_PROCESS_TIMEOUT_SECONDS:-900}"
MAX_ALLOWED_INITIAL_IMU_DROPS="${MAX_ALLOWED_INITIAL_IMU_DROPS:-1}"
# Default remains strict. A known gap already present in a recording may be
# admitted explicitly for regression runs; replay-created gaps still fail the
# completed+dropped accounting below.
MAX_ALLOWED_RECORDED_IMU_GAP_DROPS="${MAX_ALLOWED_RECORDED_IMU_GAP_DROPS:-0}"
OFFLINE_RVIZ="${OFFLINE_RVIZ:-false}"
OFFLINE_RVIZ_CONFIG="${OFFLINE_RVIZ_CONFIG:-$WORKSPACE_DIR/install/nav_bringup/share/nav_bringup/rviz/online_loop_path_compare.rviz}"
GROUND_HEIGHT_CONTINUITY_ENABLE="${GROUND_HEIGHT_CONTINUITY_ENABLE:-false}"
LOOP_POST_RESIDUAL_REFINEMENT_ENABLE="${LOOP_POST_RESIDUAL_REFINEMENT_ENABLE:-true}"

if [ -z "$BAG_DIR" ] || [ -z "$SCENE_DIR" ]; then
  echo "用法: $0 BAG_DIR SCENE_DIR [PLAY_RATE]" >&2
  exit 2
fi

BAG_DIR="$(realpath "$BAG_DIR")"
SCENE_DIR="$(realpath -m "$SCENE_DIR")"
QOS_FILE="$WORKSPACE_DIR/install/nav_bringup/share/nav_bringup/config/offline_bag_qos.yaml"
LAUNCH_LOG="$SCENE_DIR/mapping.log"
BAG_LOG="$SCENE_DIR/bag_play.log"
LAUNCH_PID=""
PAUSE_RESPONSE=""
SUPERLIO_PID=""
INPUT_QUALITY_WARNING=0

case "$PLAY_RATE" in
  ''|*[!0-9.]*)
    echo "PLAY_RATE 必须是正数: $PLAY_RATE" >&2
    exit 2
    ;;
esac
if ! [[ "$OFFLINE_PROCESS_TIMEOUT_SECONDS" =~ ^[0-9]+$ ]] ||
   ! [[ "$MAX_ALLOWED_INITIAL_IMU_DROPS" =~ ^[0-9]+$ ]] ||
   ! [[ "$MAX_ALLOWED_RECORDED_IMU_GAP_DROPS" =~ ^[0-9]+$ ]]; then
  echo "离线处理超时、允许的包头 IMU 缺失数和已知录包 IMU 间隙数必须是非负整数" >&2
  exit 2
fi
if [ "$OFFLINE_RVIZ" != "true" ] && [ "$OFFLINE_RVIZ" != "false" ]; then
  echo "OFFLINE_RVIZ 必须是 true 或 false: $OFFLINE_RVIZ" >&2
  exit 2
fi
if [ "$GROUND_HEIGHT_CONTINUITY_ENABLE" != "true" ] &&
   [ "$GROUND_HEIGHT_CONTINUITY_ENABLE" != "false" ]; then
  echo "GROUND_HEIGHT_CONTINUITY_ENABLE 必须是 true 或 false: $GROUND_HEIGHT_CONTINUITY_ENABLE" >&2
  exit 2
fi
if [ "$LOOP_POST_RESIDUAL_REFINEMENT_ENABLE" != "true" ] &&
   [ "$LOOP_POST_RESIDUAL_REFINEMENT_ENABLE" != "false" ]; then
  echo "LOOP_POST_RESIDUAL_REFINEMENT_ENABLE 必须是 true 或 false: $LOOP_POST_RESIDUAL_REFINEMENT_ENABLE" >&2
  exit 2
fi

mkdir -p "$SCENE_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 60); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

find_descendant_by_cmdline() {
  local parent_pid="$1"
  local needle="$2"
  local child_pid=""
  local match=""

  while IFS= read -r child_pid; do
    [ -n "$child_pid" ] || continue
    match="$(tr '\0' ' ' < "/proc/$child_pid/cmdline" 2>/dev/null || true)"
    if [[ "$match" == *"$needle"* ]]; then
      printf '%s\n' "$child_pid"
      return 0
    fi
    if find_descendant_by_cmdline "$child_pid" "$needle"; then
      return 0
    fi
  done < <(pgrep -P "$parent_pid" 2>/dev/null || true)
  return 1
}

expected_lidar="$(
  ros2 bag info "$BAG_DIR" |
    awk '/Topic: \/livox\/lidar / {
      for (i = 1; i <= NF; ++i) {
        if ($i == "Count:") {
          print $(i + 1)
          exit
        }
      }
    }'
)"
if ! [[ "$expected_lidar" =~ ^[0-9]+$ ]] || [ "$expected_lidar" -eq 0 ]; then
  echo "无法读取包内 /livox/lidar 帧数" >&2
  exit 1
fi

echo "离线建图: bag=$BAG_DIR scene=$SCENE_DIR rate=$PLAY_RATE expected_lidar=$expected_lidar"

setsid stdbuf -oL -eL ros2 launch nav_bringup mapping.launch.py \
  map_dir:="$SCENE_DIR" \
  map_name:=map.pcd \
  launch_livox:=false \
  launch_lio:=true \
  launch_terrain:=true \
  publish_base_footprint_tf:=true \
  rviz:="$OFFLINE_RVIZ" \
  rviz_config:="$OFFLINE_RVIZ_CONFIG" \
  offline_bag:=true \
  use_sim_time:=true \
  imu_qos_depth:=16384 \
  lidar_qos_depth:=1024 \
  pause_drain_timeout_seconds:=30.0 \
  map_preview_ds_size:=0.5 \
  map_preview_max_points:=50000 \
  ground_height_continuity_enable:="$GROUND_HEIGHT_CONTINUITY_ENABLE" \
  loop_post_residual_refinement:="$LOOP_POST_RESIDUAL_REFINEMENT_ENABLE" \
  loop_search_radius:="$LOOP_SEARCH_RADIUS" \
  fastdds_builtin_transports:=DEFAULT \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

ready=0
for _ in $(seq 1 90); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "建图进程提前退出" >&2
    tail -100 "$LAUNCH_LOG" >&2
    exit 1
  fi
  if [ "$(ros2 service type /save_terrain_map 2>/dev/null || true)" = "std_srvs/srv/Trigger" ]; then
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "建图服务未在 90 秒内就绪" >&2
  exit 1
fi

ros2 bag play "$BAG_DIR" \
  --rate "$PLAY_RATE" \
  --clock 100 \
  --delay 3 \
  --disable-keyboard-controls \
  --read-ahead-queue-size 20000 \
  --qos-profile-overrides-path "$QOS_FILE" \
  >"$BAG_LOG" 2>&1

# Reliable DDS acknowledgement happens before the subscription callback runs.
# Wait for every frame to finish LIO, not merely enter its callback/queue.
# This keeps accelerated replay from reporting success with an unfinished tail.
status_received_lidar=""
status_admitted_lidar=""
status_completed_lidar=""
status_dropped_lidar=""
status_missing_imu_start=""
status_imu_gap=""
status_invalid_time=""
status_out_of_order=""
status_pending_lidar=""
status_processing=""
for _ in $(seq 1 "$OFFLINE_PROCESS_TIMEOUT_SECONDS"); do
  STATUS_RESPONSE="$(
    ros2 service call /lio/mapping_status std_srvs/srv/Trigger "{}" 2>/dev/null ||
      true
  )"
  status_received_lidar="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*received_lidar=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_admitted_lidar="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*admitted_lidar=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_completed_lidar="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*completed_lidar=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_dropped_lidar="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*dropped_lidar=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_missing_imu_start="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*missing_imu_start=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_imu_gap="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*imu_gap=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_invalid_time="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*invalid_time=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_out_of_order="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*out_of_order=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_pending_lidar="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*pending_lidar=\([0-9][0-9]*\).*/\1/p' |
      tail -1
  )"
  status_processing="$(
    printf '%s\n' "$STATUS_RESPONSE" |
      sed -n 's/.*processing=\(true\|false\).*/\1/p' |
      tail -1
  )"
  if [[ "$status_completed_lidar" =~ ^[0-9]+$ ]] &&
     [[ "$status_dropped_lidar" =~ ^[0-9]+$ ]] &&
     [[ "$status_admitted_lidar" =~ ^[0-9]+$ ]]; then
    status_terminal_lidar=$((status_completed_lidar + status_dropped_lidar))
    if [ "$status_received_lidar" = "$expected_lidar" ] &&
       [ "$status_admitted_lidar" = "$expected_lidar" ] &&
       [ "$status_terminal_lidar" -eq "$status_admitted_lidar" ] &&
       [ "$status_pending_lidar" = "0" ] &&
       [ "$status_processing" = "false" ]; then
      break
    fi
  fi
  sleep 1
done
status_terminal_lidar=-1
status_expected_sync_drops=-1
if [[ "$status_completed_lidar" =~ ^[0-9]+$ ]] &&
   [[ "$status_dropped_lidar" =~ ^[0-9]+$ ]]; then
  status_terminal_lidar=$((status_completed_lidar + status_dropped_lidar))
fi
if [[ "$status_missing_imu_start" =~ ^[0-9]+$ ]] &&
   [[ "$status_imu_gap" =~ ^[0-9]+$ ]]; then
  status_expected_sync_drops=$((status_missing_imu_start + status_imu_gap))
fi
if [ "$status_received_lidar" != "$expected_lidar" ] ||
   [ "$status_admitted_lidar" != "$expected_lidar" ] ||
   [ "$status_terminal_lidar" -ne "$expected_lidar" ] ||
   [ "$status_pending_lidar" != "0" ] ||
   [ "$status_processing" != "false" ]; then
  INPUT_QUALITY_WARNING=1
  echo "警告: LiDAR 处理未通过完整性验收，将继续排空、回环和保存: expected=$expected_lidar received=${status_received_lidar:-unknown} admitted=${status_admitted_lidar:-unknown} completed=${status_completed_lidar:-unknown} dropped=${status_dropped_lidar:-unknown} missing_imu_start=${status_missing_imu_start:-unknown} imu_gap=${status_imu_gap:-unknown} invalid_time=${status_invalid_time:-unknown} out_of_order=${status_out_of_order:-unknown} pending=${status_pending_lidar:-unknown} processing=${status_processing:-unknown}" >&2
fi
if [ "${status_missing_imu_start:-999999}" -gt "$MAX_ALLOWED_INITIAL_IMU_DROPS" ] ||
   [ "${status_imu_gap:-999999}" -gt "$MAX_ALLOWED_RECORDED_IMU_GAP_DROPS" ] ||
   [ "$status_dropped_lidar" != "$status_expected_sync_drops" ] ||
   [ "$status_invalid_time" != "0" ] ||
   [ "$status_out_of_order" != "0" ]; then
  INPUT_QUALITY_WARNING=1
  echo "警告: 录包/同步质量未通过严格门限，但不会阻止回环保存: dropped=${status_dropped_lidar:-unknown} missing_imu_start=${status_missing_imu_start:-unknown}/${MAX_ALLOWED_INITIAL_IMU_DROPS} imu_gap=${status_imu_gap:-unknown}/${MAX_ALLOWED_RECORDED_IMU_GAP_DROPS} invalid_time=${status_invalid_time:-unknown} out_of_order=${status_out_of_order:-unknown}" >&2
fi

PAUSE_RESPONSE="$(
  ros2 service call /lio/pause_mapping std_srvs/srv/Trigger "{}" || true
)"
printf '%s\n' "$PAUSE_RESPONSE" >>"$LAUNCH_LOG"
sleep 2

pause_success="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*success[=:][[:space:]]*\([Tt]rue\|[Ff]alse\).*/\1/p' |
    tr '[:upper:]' '[:lower:]' |
    tail -1
)"
drained="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*drained=\(true\|false\).*/\1/p' |
    tail -1
)"
received_lidar="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*received_lidar=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
admitted_lidar="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*admitted_lidar=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
completed_lidar="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*completed_lidar=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
dropped_lidar="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*dropped_lidar=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
missing_imu_start="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*missing_imu_start=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
imu_gap="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*imu_gap=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
invalid_time="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*invalid_time=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
out_of_order="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*out_of_order=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
terminal_lidar=-1
expected_sync_drops=-1
if [[ "$completed_lidar" =~ ^[0-9]+$ ]] &&
   [[ "$dropped_lidar" =~ ^[0-9]+$ ]]; then
  terminal_lidar=$((completed_lidar + dropped_lidar))
fi
if [[ "$missing_imu_start" =~ ^[0-9]+$ ]] &&
   [[ "$imu_gap" =~ ^[0-9]+$ ]]; then
  expected_sync_drops=$((missing_imu_start + imu_gap))
fi
source_gaps="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*source_gaps=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
if [ "$pause_success" != "true" ] ||
   [ "$drained" != "true" ] ||
   [ "$received_lidar" != "$expected_lidar" ] ||
   [ "$admitted_lidar" != "$expected_lidar" ] ||
   [ "$terminal_lidar" -ne "$expected_lidar" ] ||
   [ "${missing_imu_start:-999999}" -gt "$MAX_ALLOWED_INITIAL_IMU_DROPS" ] ||
   [ "${imu_gap:-999999}" -gt "$MAX_ALLOWED_RECORDED_IMU_GAP_DROPS" ] ||
   [ "$dropped_lidar" != "$expected_sync_drops" ] ||
   [ "$invalid_time" != "0" ] ||
   [ "$out_of_order" != "0" ]; then
  INPUT_QUALITY_WARNING=1
  echo "警告: 离线建图验收未完全通过，将继续回环和保存: pause_success=${pause_success:-unknown} drained=${drained:-unknown} expected=$expected_lidar received=${received_lidar:-unknown} admitted=${admitted_lidar:-unknown} completed=${completed_lidar:-unknown} dropped=${dropped_lidar:-unknown} missing_imu_start=${missing_imu_start:-unknown} imu_gap=${imu_gap:-unknown} invalid_time=${invalid_time:-unknown} out_of_order=${out_of_order:-unknown}" >&2
fi
# source_gaps may already exist in the recording. Exact received/admitted/
# completed equality plus zero processing drops is the replay-completeness
# criterion.

SUPERLIO_PID="$(find_descendant_by_cmdline "$LAUNCH_PID" "super_lio_node" || true)"
if [ -z "$SUPERLIO_PID" ]; then
  echo "无法定位 SuperLIO 进程，不能在 terrain 保存前生成闭环校正" >&2
  exit 1
fi
kill -INT "$SUPERLIO_PID" 2>/dev/null || true
for _ in $(seq 1 "$SUPERLIO_SAVE_TIMEOUT_SECONDS"); do
  kill -0 "$SUPERLIO_PID" 2>/dev/null || break
  sleep 1
done
if kill -0 "$SUPERLIO_PID" 2>/dev/null; then
  echo "SuperLIO 保存闭环地图超时" >&2
  exit 1
fi

# SuperLIO writes loop_pose_graph.txt before exiting. Terrain must save after
# that point so ground/footprint receive the exact same time-varying SE(3)
# correction as the canonical map.pcd.
ros2 service call /save_terrain_map std_srvs/srv/Trigger "{}" >>"$LAUNCH_LOG" 2>&1

kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
for _ in $(seq 1 60); do
  kill -0 "$LAUNCH_PID" 2>/dev/null || break
  sleep 1
done
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

ground_src="$(
  find "$SCENE_DIR" -maxdepth 1 -type f -name '*_ground.pcd' \
    -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-
)"
footprint_src="$(
  find "$SCENE_DIR" -maxdepth 1 -type f -name '*_base_footprint_fill.pcd' \
    -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-
)"
[ -s "$SCENE_DIR/map.pcd" ]
[ -n "$ground_src" ] && [ -s "$ground_src" ]
[ -n "$footprint_src" ] && [ -s "$footprint_src" ]
cp -f "$ground_src" "$SCENE_DIR/ground.pcd"
cp -f "$footprint_src" "$SCENE_DIR/footprint.pcd"
# BotDog 场景扫描按文件名后缀 `footprint_fill.pcd` 识别足迹填充层。
# `footprint.pcd` 保留给现有 ROS/调试工具；两者内容相同，但规范名称
# 不能省略，否则前端会把场景显示为“缺少足迹填充”。
cp -f "$footprint_src" "$SCENE_DIR/footprint_fill.pcd"

if [ "$INPUT_QUALITY_WARNING" -ne 0 ]; then
  cat >"$SCENE_DIR/offline_validation_warning.txt" <<EOF
Offline mapping completed and artifacts were saved, but input validation reported a warning.
expected_lidar=$expected_lidar
received_lidar=${received_lidar:-unknown}
admitted_lidar=${admitted_lidar:-unknown}
completed_lidar=${completed_lidar:-unknown}
dropped_lidar=${dropped_lidar:-unknown}
missing_imu_start=${missing_imu_start:-unknown}
imu_gap=${imu_gap:-unknown}
invalid_time=${invalid_time:-unknown}
out_of_order=${out_of_order:-unknown}
source_gaps=${source_gaps:-unknown}
EOF
fi

echo "离线建图完成: validation_warning=$INPUT_QUALITY_WARNING received_lidar=$received_lidar admitted_lidar=$admitted_lidar completed_lidar=$completed_lidar dropped_lidar=$dropped_lidar missing_imu_start=$missing_imu_start imu_gap=$imu_gap invalid_time=$invalid_time out_of_order=$out_of_order source_gaps=$source_gaps"
stat -c '%n %s bytes' \
  "$SCENE_DIR/map.pcd" \
  "$SCENE_DIR/ground.pcd" \
  "$SCENE_DIR/footprint.pcd" \
  "$SCENE_DIR/footprint_fill.pcd"
