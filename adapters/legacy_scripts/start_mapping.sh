#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

MAP_DIR="${1:-}"
if [ -z "$MAP_DIR" ]; then
  log_error "缺少地图保存目录参数"
  log_error "用法：bash start_mapping.sh /path/to/maps/Scene001"
  exit 1
fi

MAP_DIR="$(realpath -m "$MAP_DIR")"
LOG_FILE="$ROBOT_NAV_LOG_ROOT/start_mapping_debug.log"
PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/mapping.pid"
READY_FILE="$MAP_DIR/.ground_generation_started"
STATUS_FILE="$ROBOT_NAV_RUNTIME_ROOT/mapping_status.json"
MAPPING_READY_TIMEOUT_SECONDS="${MAPPING_READY_TIMEOUT_SECONDS:-55}"
MAPPING_LIVOX_DATA_TIMEOUT_SECONDS="${MAPPING_LIVOX_DATA_TIMEOUT_SECONDS:-8}"
SUPERLIO_SAVE_TIMEOUT_SECONDS="${SUPERLIO_SAVE_TIMEOUT_SECONDS:-100}"
SUPERLIO_PAUSE_TIMEOUT_SECONDS="${SUPERLIO_PAUSE_TIMEOUT_SECONDS:-30}"
LAUNCH_STOP_TIMEOUT_SECONDS="${LAUNCH_STOP_TIMEOUT_SECONDS:-15}"
LAUNCH_TERM_TIMEOUT_SECONDS="${LAUNCH_TERM_TIMEOUT_SECONDS:-10}"
RUN_LOG_OFFSET=0
CLEANING_UP=0
LAUNCH_PGID=""
MAPPING_FAILURE_MESSAGE=""

mkdir -p "$MAP_DIR"
rm -f "$READY_FILE" "$PID_FILE"

source_navigation_env
prepare_livox_config

process_is_running() {
  local pid="$1"
  local state=""

  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
  [ -n "$state" ] && [[ "$state" != Z* ]]
}

wait_for_process_exit() {
  local pid="$1"
  local timeout_seconds="$2"
  local waited=0

  while process_is_running "$pid" && [ "$waited" -lt "$timeout_seconds" ]; do
    sleep 1
    waited=$((waited + 1))
  done

  ! process_is_running "$pid"
}

list_descendant_pids() {
  local parent_pid="$1"
  local child_pid=""

  while IFS= read -r child_pid; do
    [ -n "$child_pid" ] || continue
    printf '%s\n' "$child_pid"
    list_descendant_pids "$child_pid"
  done < <(pgrep -P "$parent_pid" 2>/dev/null || true)
}

find_launch_descendant() {
  local needle="$1"
  local child_pid=""
  local cmdline=""

  while IFS= read -r child_pid; do
    [ -n "$child_pid" ] || continue
    cmdline="$(tr '\0' ' ' < "/proc/$child_pid/cmdline" 2>/dev/null || true)"
    if [[ "$cmdline" == *"$needle"* ]]; then
      printf '%s\n' "$child_pid"
      return 0
    fi
  done < <(list_descendant_pids "$LAUNCH_PID")

  return 1
}

launch_group_is_running() {
  if [ -n "$LAUNCH_PGID" ]; then
    ps -eo pgid=,stat= | awk -v pgid="$LAUNCH_PGID" '
      $1 == pgid && $2 !~ /^Z/ { found = 1 }
      END { exit(found ? 0 : 1) }
    '
    return $?
  fi

  process_is_running "$LAUNCH_PID"
}

wait_for_launch_exit() {
  local timeout_seconds="$1"
  local waited=0

  while launch_group_is_running && [ "$waited" -lt "$timeout_seconds" ]; do
    sleep 1
    waited=$((waited + 1))
  done

  ! launch_group_is_running
}

signal_launch_tree() {
  local signal_name="$1"
  local current_pgid=""
  local child_pid=""

  current_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d '[:space:]' || true)"
  if [ -n "$LAUNCH_PGID" ] && [ "$LAUNCH_PGID" != "$current_pgid" ]; then
    kill "-$signal_name" -- "-$LAUNCH_PGID" 2>/dev/null || true
    return 0
  fi

  # setsid 不可用时不能向当前进程组发信号，否则会杀掉清理脚本自身。
  while IFS= read -r child_pid; do
    [ -n "$child_pid" ] || continue
    kill "-$signal_name" "$child_pid" 2>/dev/null || true
  done < <(list_descendant_pids "$LAUNCH_PID")
  kill "-$signal_name" "$LAUNCH_PID" 2>/dev/null || true
}

cleanup() {
  local status="${1:-0}"
  local superlio_pid=""
  local superlio_paused=0
  local launch_wait_seconds="$LAUNCH_STOP_TIMEOUT_SECONDS"

  if [ "$CLEANING_UP" -eq 1 ]; then
    return 0
  fi
  CLEANING_UP=1
  trap - INT TERM EXIT

  if [ -n "${LAUNCH_PID:-}" ] && launch_group_is_running; then
    log_info "收到停止信号，先冻结 SuperLIO，再保存 map.pcd 和 terrain_map"
    superlio_pid="$(find_launch_descendant "super_lio_node" || true)"
    if [ -n "$superlio_pid" ] && process_is_running "$superlio_pid"; then
      if request_pause_super_lio "$LOG_FILE" "$SUPERLIO_PAUSE_TIMEOUT_SECONDS"; then
        superlio_paused=1
        # 让 terrain 节点消费完冻结前已经送达的最后一帧。
        sleep 1
      else
        log_warn "无法通过服务冻结 SuperLIO，将直接优雅停止，避免保存期间继续建图"
      fi

      log_info "请求 SuperLIO 优雅退出并保存 map.pcd：PID=$superlio_pid"
      kill -INT "$superlio_pid" 2>/dev/null || true
      if wait_for_process_exit "$superlio_pid" "$SUPERLIO_SAVE_TIMEOUT_SECONDS"; then
        log_info "SuperLIO 已退出，map.pcd 保存阶段完成"
      else
        log_warn "SuperLIO 在 ${SUPERLIO_SAVE_TIMEOUT_SECONDS}s 内未退出，将继续关闭 launch 进程组"
      fi
    else
      # 找不到节点时仍优雅停止整个隔离进程组；SIGINT 会直接送达 ROS 节点。
      log_warn "未找到 SuperLIO 子进程，改为向建图 launch 进程组发送 SIGINT"
      signal_launch_tree INT
      launch_wait_seconds="$SUPERLIO_SAVE_TIMEOUT_SECONDS"
    fi

    if [ -s "$MAP_DIR/map.pcd" ]; then
      log_info "SuperLIO 地图已保存：$MAP_DIR/map.pcd"
    else
      log_warn "SuperLIO 优雅退出后仍未发现 map.pcd：$MAP_DIR/map.pcd"
    fi

    if [ "$superlio_paused" -eq 1 ]; then
      log_info "基于已冻结的最终建图状态保存 terrain_map、ground 和 footprint 点云"
    else
      log_info "基于 SuperLIO 停止前 terrain 节点已累计的数据保存 terrain_map"
    fi
    request_save_terrain_map "$LOG_FILE" || true

    if launch_group_is_running; then
      signal_launch_tree INT
    fi
    if ! wait_for_launch_exit "$launch_wait_seconds"; then
      log_warn "建图 launch 在 ${launch_wait_seconds}s 内未退出，发送 SIGTERM"
      signal_launch_tree TERM
      if ! wait_for_launch_exit "$LAUNCH_TERM_TIMEOUT_SECONDS"; then
        log_warn "建图 launch 仍未退出，发送 SIGKILL"
        signal_launch_tree KILL
        wait_for_launch_exit 5 || true
      fi
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  rm -f "$PID_FILE"
  if [ -n "$MAPPING_FAILURE_MESSAGE" ]; then
    write_json_file "$STATUS_FILE" "{\"running\":false,\"map_dir\":\"$MAP_DIR\",\"stage\":\"error\",\"exit_code\":$status,\"message\":\"$MAPPING_FAILURE_MESSAGE\"}"
  else
    write_json_file "$STATUS_FILE" "{\"running\":false,\"map_dir\":\"$MAP_DIR\",\"exit_code\":$status}"
  fi
  exit "$status"
}

trap 'cleanup 0' INT TERM
trap 'cleanup $?' EXIT

log_info "开始建图：$MAP_DIR"
log_info "日志文件：$LOG_FILE"
log_info "Livox 雷达 IP：$LIVOX_LIDAR_IP"
log_info "Livox 配置文件：$LIVOX_CONFIG_PATH"
log_info "雷达安装标定(base_footprint->lidar)：xyz=[$NAV_LIDAR_MOUNT_X_M,$NAV_LIDAR_MOUNT_Y_M,$NAV_LIDAR_MOUNT_Z_M]m rpy=[$NAV_LIDAR_MOUNT_ROLL_DEG,$NAV_LIDAR_MOUNT_PITCH_DEG,$NAV_LIDAR_MOUNT_YAW_DEG]deg"
if [ -n "$LIVOX_HOST_IP" ]; then
  log_info "本机雷达网卡 IP：$LIVOX_HOST_IP"
else
  log_warn "未设置 LIVOX_HOST_IP，请确认本机网卡与雷达同网段"
fi

write_json_file "$STATUS_FILE" "{\"running\":true,\"map_dir\":\"$MAP_DIR\",\"stage\":\"starting\"}"

launch_args=(
  "map_dir:=$MAP_DIR"
  "lidar_ip:=$LIVOX_LIDAR_IP"
  "livox_config_path:=$LIVOX_CONFIG_PATH"
  "publish_base_footprint_tf:=true"
  "lidar_mount_x_m:=$NAV_LIDAR_MOUNT_X_M"
  "lidar_mount_y_m:=$NAV_LIDAR_MOUNT_Y_M"
  "lidar_mount_z_m:=$NAV_LIDAR_MOUNT_Z_M"
  "lidar_mount_roll_deg:=$NAV_LIDAR_MOUNT_ROLL_DEG"
  "lidar_mount_pitch_deg:=$NAV_LIDAR_MOUNT_PITCH_DEG"
  "lidar_mount_yaw_deg:=$NAV_LIDAR_MOUNT_YAW_DEG"
)

if [ -n "$LIVOX_HOST_IP" ]; then
  launch_args+=("host_ip:=$LIVOX_HOST_IP")
fi

RUN_LOG_OFFSET="$(stat -c '%s' "$LOG_FILE" 2>/dev/null || printf '0')"
if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch nav_bringup mapping.launch.py "${launch_args[@]}" >> "$LOG_FILE" 2>&1 &
else
  log_warn "系统缺少 setsid，建图 launch 无法创建独立进程组"
  ros2 launch nav_bringup mapping.launch.py "${launch_args[@]}" >> "$LOG_FILE" 2>&1 &
fi

LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d '[:space:]' || true)"
printf '%s\n' "$LAUNCH_PID" > "$PID_FILE"

mapping_log_has() {
  local marker="$1"
  grep -Fq "$marker" < <(tail -c "+$((RUN_LOG_OFFSET + 1))" "$LOG_FILE" 2>/dev/null)
}

mapping_service_ready() {
  [ "$(ros2 service type /save_terrain_map 2>/dev/null || true)" = "std_srvs/srv/Trigger" ]
}

wait_for_livox_data() {
  # topic 由本脚本启动的 Livox 驱动创建；只有收到真实帧才允许写 ready。
  # `ros2 topic echo` 在 Humble 支持 sensor_data QoS，field 可避免展开整帧点云。
  timeout "${MAPPING_LIVOX_DATA_TIMEOUT_SECONDS}s" \
    ros2 topic echo /livox/lidar --once --qos-profile sensor_data --field timebase \
    >/dev/null 2>&1
}

ready_waited=0
while kill -0 "$LAUNCH_PID" 2>/dev/null && [ "$ready_waited" -lt "$MAPPING_READY_TIMEOUT_SECONDS" ]; do
  if mapping_log_has "publish use livox custom format" \
    && mapping_log_has "Map init done" \
    && mapping_service_ready; then
    break
  fi
  sleep 1
  ready_waited=$((ready_waited + 1))
done

if kill -0 "$LAUNCH_PID" 2>/dev/null \
  && mapping_log_has "publish use livox custom format" \
  && mapping_log_has "Map init done" \
  && mapping_service_ready; then
  if ! wait_for_livox_data; then
    MAPPING_FAILURE_MESSAGE="雷达无有效数据：${MAPPING_LIVOX_DATA_TIMEOUT_SECONDS} 秒内未收到 /livox/lidar 点云，请检查 Livox MID360 供电、网线和 IP 配置"
    log_error "$MAPPING_FAILURE_MESSAGE"
    exit 1
  fi
  date '+%Y-%m-%d %H:%M:%S' > "$READY_FILE"
  write_json_file "$STATUS_FILE" "{\"running\":true,\"map_dir\":\"$MAP_DIR\",\"pid\":$LAUNCH_PID,\"stage\":\"running\"}"
  log_info "建图进程已启动：PID=$LAUNCH_PID"
  log_info "ready 文件：$READY_FILE"
else
  log_error "建图链路在 ${MAPPING_READY_TIMEOUT_SECONDS}s 内未就绪，请检查 Livox、SuperLIO 和 terrain 服务：$LOG_FILE"
  exit 1
fi

wait "$LAUNCH_PID"
