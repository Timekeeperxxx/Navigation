#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

SCENE_DIR="${1:-}"
if [ -z "$SCENE_DIR" ]; then
  log_error "缺少场景目录参数"
  log_error "用法：bash restart_navigation_localization.sh /path/to/maps/Scene001"
  exit 1
fi

SCENE_DIR="$(realpath -m "$SCENE_DIR")"
if [ ! -d "$SCENE_DIR" ]; then
  log_error "场景目录不存在：$SCENE_DIR"
  exit 1
fi

LOG_FILE="$ROBOT_NAV_LOG_ROOT/restart_navigation_localization.log"
PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation.pid"
READY_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation_ready.json"
STATUS_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation_status.json"

MAP_PCD="$(find_scene_pcd_file "$SCENE_DIR" "map.pcd" "map.pcd" "map.pcd")"
GROUND_PCD="$(find_scene_pcd_file "$SCENE_DIR" "ground.pcd" "*ground.pcd" "ground.pcd")"
PLANGROUND_PCD="$(find_scene_pcd_file "$SCENE_DIR" "footprint_fill.pcd" "*footprint_fill.pcd" "footprint_fill.pcd" || true)"
NAV_LIO_MAP_DIR="$(super_lio_relative_map_dir "$MAP_PCD")"
MAP_NAME="$(basename "$MAP_PCD")"

rm -f "$PID_FILE" "$READY_FILE"
source_navigation_env
prepare_livox_config

cleanup() {
  local status=$?
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log_info "收到停止信号，正在停止导航定位链路"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  rm -f "$PID_FILE"
  write_json_file "$STATUS_FILE" "{\"running\":false,\"scene_dir\":\"$SCENE_DIR\",\"exit_code\":$status}"
  exit "$status"
}

trap cleanup INT TERM EXIT

log_info "启动导航定位链路：$SCENE_DIR"
log_info "map.pcd：$MAP_PCD"
log_info "ground.pcd：$GROUND_PCD"
log_info "Livox 雷达 IP：$LIVOX_LIDAR_IP"
log_info "Livox 配置文件：$LIVOX_CONFIG_PATH"
if [ -n "$LIVOX_HOST_IP" ]; then
  log_info "本机雷达网卡 IP：$LIVOX_HOST_IP"
else
  log_warn "未设置 LIVOX_HOST_IP，请确认本机网卡与雷达同网段"
fi
if [ -n "$PLANGROUND_PCD" ]; then
  log_info "footprint_fill.pcd：$PLANGROUND_PCD"
else
  log_warn "未找到 footprint_fill.pcd，将不传入 planground"
fi

write_json_file "$STATUS_FILE" "{\"running\":true,\"scene_dir\":\"$SCENE_DIR\",\"stage\":\"starting\"}"

launch_args=(
  "scene_dir:=$SCENE_DIR"
  "nav_lio_map_dir:=$NAV_LIO_MAP_DIR"
  "map_name:=$MAP_NAME"
  "map_pcd:=$MAP_PCD"
  "ground_pcd:=$GROUND_PCD"
  "launch_livox:=true"
  "lidar_ip:=$LIVOX_LIDAR_IP"
  "livox_config_path:=$LIVOX_CONFIG_PATH"
  "rviz:=false"
)

if [ -n "$LIVOX_HOST_IP" ]; then
  launch_args+=("host_ip:=$LIVOX_HOST_IP")
fi

if [ -n "$PLANGROUND_PCD" ]; then
  launch_args+=("planground_pcd:=$PLANGROUND_PCD")
fi

ros2 launch nav_bringup navigation.launch.py "${launch_args[@]}" >> "$LOG_FILE" 2>&1 &

LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" > "$PID_FILE"

sleep 3
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
  write_json_file "$READY_FILE" "{\"ready\":true,\"scene_dir\":\"$SCENE_DIR\",\"map_pcd\":\"$MAP_PCD\",\"ground_pcd\":\"$GROUND_PCD\",\"planground_pcd\":\"$PLANGROUND_PCD\",\"pid\":$LAUNCH_PID}"
  write_json_file "$STATUS_FILE" "{\"running\":true,\"scene_dir\":\"$SCENE_DIR\",\"pid\":$LAUNCH_PID,\"stage\":\"running\"}"
  log_info "导航定位进程已启动：PID=$LAUNCH_PID"
  log_info "ready 文件：$READY_FILE"
else
  log_error "导航定位进程启动后立即退出，请查看日志：$LOG_FILE"
  exit 1
fi

wait "$LAUNCH_PID"
