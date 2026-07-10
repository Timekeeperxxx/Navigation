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
RUN_LOG_OFFSET=0

mkdir -p "$MAP_DIR"
rm -f "$READY_FILE" "$PID_FILE"

source_navigation_env
prepare_livox_config

cleanup() {
  local status=$?
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log_info "收到停止信号，正在请求保存 terrain_map 并停止建图"
    request_save_terrain_map "$LOG_FILE" || true
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  rm -f "$PID_FILE"
  write_json_file "$STATUS_FILE" "{\"running\":false,\"map_dir\":\"$MAP_DIR\",\"exit_code\":$status}"
  exit "$status"
}

trap cleanup INT TERM EXIT

log_info "开始建图：$MAP_DIR"
log_info "日志文件：$LOG_FILE"
log_info "Livox 雷达 IP：$LIVOX_LIDAR_IP"
log_info "Livox 配置文件：$LIVOX_CONFIG_PATH"
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
)

if [ -n "$LIVOX_HOST_IP" ]; then
  launch_args+=("host_ip:=$LIVOX_HOST_IP")
fi

RUN_LOG_OFFSET="$(stat -c '%s' "$LOG_FILE" 2>/dev/null || printf '0')"
ros2 launch nav_bringup mapping.launch.py "${launch_args[@]}" >> "$LOG_FILE" 2>&1 &

LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" > "$PID_FILE"

mapping_log_has() {
  local marker="$1"
  grep -Fq "$marker" < <(tail -c "+$((RUN_LOG_OFFSET + 1))" "$LOG_FILE" 2>/dev/null)
}

mapping_service_ready() {
  [ "$(ros2 service type /save_terrain_map 2>/dev/null || true)" = "std_srvs/srv/Trigger" ]
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
  date '+%Y-%m-%d %H:%M:%S' > "$READY_FILE"
  write_json_file "$STATUS_FILE" "{\"running\":true,\"map_dir\":\"$MAP_DIR\",\"pid\":$LAUNCH_PID,\"stage\":\"running\"}"
  log_info "建图进程已启动：PID=$LAUNCH_PID"
  log_info "ready 文件：$READY_FILE"
else
  log_error "建图链路在 ${MAPPING_READY_TIMEOUT_SECONDS}s 内未就绪，请检查 Livox、SuperLIO 和 terrain 服务：$LOG_FILE"
  exit 1
fi

wait "$LAUNCH_PID"
