#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${1:-}"
SCENE_DIR="${2:-}"
PLAY_RATE="${3:-0.5}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

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

case "$PLAY_RATE" in
  ''|*[!0-9.]*)
    echo "PLAY_RATE 必须是正数: $PLAY_RATE" >&2
    exit 2
    ;;
esac

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
  rviz:=false \
  offline_bag:=true \
  use_sim_time:=true \
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
  --read-ahead-queue-size 2000 \
  --qos-profile-overrides-path "$QOS_FILE" \
  >"$BAG_LOG" 2>&1

# Reliable DDS acknowledgement happens before the subscription callback runs.
# Give the deep offline queues time to drain before freezing the final state.
sleep 10
PAUSE_RESPONSE="$(
  ros2 service call /lio/pause_mapping std_srvs/srv/Trigger "{}"
)"
printf '%s\n' "$PAUSE_RESPONSE" >>"$LAUNCH_LOG"
sleep 2
ros2 service call /save_terrain_map std_srvs/srv/Trigger "{}" >>"$LAUNCH_LOG" 2>&1

received_lidar="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*received_lidar=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
source_gaps="$(
  printf '%s\n' "$PAUSE_RESPONSE" |
    sed -n 's/.*source_gaps=\([0-9][0-9]*\).*/\1/p' |
    tail -1
)"
if [ "$received_lidar" != "$expected_lidar" ]; then
  echo "离线建图验收失败: expected_lidar=$expected_lidar received_lidar=${received_lidar:-unknown}" >&2
  exit 1
fi
# source_gaps may already exist in the recording. Equality between the bag
# topic count and received_lidar is the transport-completeness criterion.

kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
for _ in $(seq 1 180); do
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

echo "离线建图完成: received_lidar=$received_lidar source_gaps=$source_gaps"
stat -c '%n %s bytes' \
  "$SCENE_DIR/map.pcd" \
  "$SCENE_DIR/ground.pcd" \
  "$SCENE_DIR/footprint.pcd"
