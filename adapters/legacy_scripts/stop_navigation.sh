#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

log_info "停止 Navigation 兼容脚本启动的进程"

source_navigation_env

if [ -f "$ROBOT_NAV_RUNTIME_ROOT/mapping.pid" ]; then
  request_save_terrain_map "$ROBOT_NAV_LOG_ROOT/start_mapping_debug.log" || true
fi

stop_pid_file "$ROBOT_NAV_RUNTIME_ROOT/mapping.pid" "建图链路" 90 "ros2 launch nav_bringup mapping.launch.py"
stop_pid_file "$ROBOT_NAV_RUNTIME_ROOT/navigation_adapter.pid" "导航适配器" 10 "restart_navigation_localization.sh"
stop_pid_file "$ROBOT_NAV_RUNTIME_ROOT/navigation.pid" "导航定位链路" 10 "ros2 launch nav_bringup navigation.launch.py"

rm -f \
  "$ROBOT_NAV_RUNTIME_ROOT/navigation_ready.json" \
  "$ROBOT_NAV_RUNTIME_ROOT/navigation_adapter.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/livox.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/relocation.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/global_planner.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/scan_planner.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/scan_controller.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/scan_tf_pose.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/dynamic_avoidance.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/nav_status_monitor.pid" \
  "$ROBOT_NAV_RUNTIME_ROOT/p2p_move_base.pid"

write_json_file "$ROBOT_NAV_RUNTIME_ROOT/navigation_status.json" "{\"running\":false,\"stage\":\"stopped\"}"
write_json_file "$ROBOT_NAV_RUNTIME_ROOT/mapping_status.json" "{\"running\":false,\"stage\":\"stopped\"}"

log_info "停止请求已完成"
