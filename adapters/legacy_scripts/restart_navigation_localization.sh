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
ADAPTER_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation_adapter.pid"
LOCK_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation.lock"
READY_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation_ready.json"
STATUS_FILE="$ROBOT_NAV_RUNTIME_ROOT/navigation_status.json"
LIVOX_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/livox.pid"
RELOCATION_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/relocation.pid"
GLOBAL_PLANNER_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/global_planner.pid"
SCAN_PLANNER_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/scan_planner.pid"
SCAN_CONTROLLER_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/scan_controller.pid"
SCAN_TF_POSE_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/scan_tf_pose.pid"
DYNAMIC_AVOIDANCE_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/dynamic_avoidance.pid"
NAV_STATUS_MONITOR_PID_FILE="$ROBOT_NAV_RUNTIME_ROOT/nav_status_monitor.pid"
NAV_ENABLE_SCAN_PLANNER="${NAV_ENABLE_SCAN_PLANNER:-true}"
NAV_ENABLE_SCAN_CONTROLLER="${NAV_ENABLE_SCAN_CONTROLLER:-true}"
NAV_ENABLE_PATH_FOLLOWER="${NAV_ENABLE_PATH_FOLLOWER:-false}"
NAV_ENABLE_DYNAMIC_AVOIDANCE="${NAV_ENABLE_DYNAMIC_AVOIDANCE:-true}"
NAV_ENABLE_ROBOT_CONTROL="${NAV_ENABLE_ROBOT_CONTROL:-false}"
NAV_ROBOT_MODEL="${NAV_ROBOT_MODEL:-b2}"
NAV_ROBOT_CMD_VEL_TOPIC="${NAV_ROBOT_CMD_VEL_TOPIC:-/unitree/b2/cmd_vel}"
NAV_GO2_CONNECTION_METHOD="${NAV_GO2_CONNECTION_METHOD:-LocalSTA}"
NAV_GO2_IP="${NAV_GO2_IP:-}"
NAV_GO2_SERIAL_NUMBER="${NAV_GO2_SERIAL_NUMBER:-}"
NAV_GO2_AES_128_KEY="${NAV_GO2_AES_128_KEY:-}"
NAV_READY_TIMEOUT_SECONDS="${NAV_READY_TIMEOUT_SECONDS:-540}"
RUN_LOG_OFFSET=0
RUN_ID="$(date +%s)-$$"

if { [ "$NAV_ENABLE_ROBOT_CONTROL" = "true" ] || [ "$NAV_ENABLE_ROBOT_CONTROL" = "1" ]; } \
  && [ "$NAV_ROBOT_MODEL" = "go2_webrtc" ] \
  && [ "$NAV_GO2_CONNECTION_METHOD" = "LocalSTA" ] \
  && [ -z "$NAV_GO2_IP" ]; then
  log_error "启用 Go2 LocalSTA 控制时必须设置 NAV_GO2_IP"
  exit 1
fi

if { [ "$NAV_ENABLE_SCAN_CONTROLLER" = "true" ] || [ "$NAV_ENABLE_SCAN_CONTROLLER" = "1" ]; } \
  && { [ "$NAV_ENABLE_PATH_FOLLOWER" = "true" ] || [ "$NAV_ENABLE_PATH_FOLLOWER" = "1" ]; }; then
  log_error "SCAN closed_loop_controller 与 nav_path_follower 不能同时启用（都会发布 /cmd_vel）"
  exit 1
fi

MAP_PCD="$(find_scene_pcd_file "$SCENE_DIR" "map.pcd" "map.pcd" "map.pcd")"
GROUND_PCD="$(find_scene_pcd_file "$SCENE_DIR" "ground.pcd" "*ground.pcd" "ground.pcd")"
PLANGROUND_PCD="$(find_scene_pcd_file "$SCENE_DIR" \
  "footprint_fill.pcd|fill_footpoint.pcd" \
  "*_base_footprint_fill.pcd|*footprint_fill.pcd|*fill_footpoint*.pcd" \
  "footprint_fill/fill_footpoint.pcd" || true)"

validate_navigation_pcd_input "$MAP_PCD" "map.pcd"
validate_navigation_pcd_input "$GROUND_PCD" "ground.pcd"
if [ -n "$PLANGROUND_PCD" ]; then
  validate_navigation_pcd_input "$PLANGROUND_PCD" "footprint_fill.pcd"
fi

NAV_LIO_MAP_DIR="$(super_lio_relative_map_dir "$MAP_PCD")"
MAP_NAME="$(basename "$MAP_PCD")"

CHILD_PID_FILES=(
  "$LIVOX_PID_FILE"
  "$RELOCATION_PID_FILE"
  "$GLOBAL_PLANNER_PID_FILE"
  "$SCAN_PLANNER_PID_FILE"
  "$SCAN_CONTROLLER_PID_FILE"
  "$SCAN_TF_POSE_PID_FILE"
  "$DYNAMIC_AVOIDANCE_PID_FILE"
  "$NAV_STATUS_MONITOR_PID_FILE"
)

source_navigation_env

exec 9> "$LOCK_FILE"
if ! flock -n 9; then
  log_warn "检测到上一轮导航适配器仍持有单实例锁，正在停止旧实例"
  stop_pid_file "$ADAPTER_PID_FILE" "旧导航适配器" 10 "restart_navigation_localization.sh"
  stop_pid_file "$PID_FILE" "旧导航定位链路" 10 "ros2 launch nav_bringup navigation.launch.py"
  if ! flock -w 20 9; then
    log_error "无法在 20 秒内取得导航单实例锁：$LOCK_FILE"
    exit 1
  fi
fi
printf '%s\n' "$$" > "$ADAPTER_PID_FILE"

stop_pid_file "$PID_FILE" "旧导航定位链路" 10 "ros2 launch nav_bringup navigation.launch.py"
rm -f "$READY_FILE" "$ROBOT_NAV_RUNTIME_ROOT/p2p_move_base.pid" "${CHILD_PID_FILES[@]}"
prepare_livox_config

record_process_pid() {
  local pid_file="$1"
  local process_pattern="$2"
  local pid=""

  pid="$(pgrep -f -- "$process_pattern" 2>/dev/null | tail -n 1 || true)"
  if [ -z "$pid" ]; then
    return 1
  fi
  printf '%s\n' "$pid" > "$pid_file"
}

record_navigation_children() {
  local missing=0

  record_process_pid "$LIVOX_PID_FILE" "/livox_ros_driver2/livox_ros_driver2_node" || missing=$((missing + 1))
  record_process_pid "$RELOCATION_PID_FILE" "/nav_lio/relocation_node" || missing=$((missing + 1))
  record_process_pid "$GLOBAL_PLANNER_PID_FILE" "/nav_planner/global_planner_node" || missing=$((missing + 1))
  record_process_pid "$NAV_STATUS_MONITOR_PID_FILE" "/nav_planner/waypoint_progress_monitor.py" || missing=$((missing + 1))

  if [ "$NAV_ENABLE_SCAN_PLANNER" = "true" ] || [ "$NAV_ENABLE_SCAN_PLANNER" = "1" ]; then
    record_process_pid "$SCAN_PLANNER_PID_FILE" "/scan_planner/scan_planner_node" || missing=$((missing + 1))
  fi
  if [ "$NAV_ENABLE_SCAN_CONTROLLER" = "true" ] || [ "$NAV_ENABLE_SCAN_CONTROLLER" = "1" ]; then
    record_process_pid "$SCAN_CONTROLLER_PID_FILE" "/scan_planner/closed_loop_controller" || missing=$((missing + 1))
  fi
  if { [ "$NAV_ENABLE_SCAN_PLANNER" = "true" ] || [ "$NAV_ENABLE_SCAN_PLANNER" = "1" ]; } \
    || { [ "$NAV_ENABLE_SCAN_CONTROLLER" = "true" ] || [ "$NAV_ENABLE_SCAN_CONTROLLER" = "1" ]; }; then
    record_process_pid "$SCAN_TF_POSE_PID_FILE" "/nav_bringup/scan_tf_pose_publisher.py" || missing=$((missing + 1))
  fi
  if [ "$NAV_ENABLE_DYNAMIC_AVOIDANCE" = "true" ] || [ "$NAV_ENABLE_DYNAMIC_AVOIDANCE" = "1" ]; then
    record_process_pid "$DYNAMIC_AVOIDANCE_PID_FILE" "/nav_planner/dynamic_avoidance_monitor.py" || missing=$((missing + 1))
  fi

  [ "$missing" -eq 0 ]
}

global_planner_map_ready() {
  grep -Fq "Published static graph with" \
    < <(tail -c "+$((RUN_LOG_OFFSET + 1))" "$LOG_FILE" 2>/dev/null)
}

scan_body_pose_ready() {
  if { [ "$NAV_ENABLE_SCAN_PLANNER" != "true" ] && [ "$NAV_ENABLE_SCAN_PLANNER" != "1" ]; } \
    && { [ "$NAV_ENABLE_SCAN_CONTROLLER" != "true" ] && [ "$NAV_ENABLE_SCAN_CONTROLLER" != "1" ]; }; then
    return 0
  fi
  grep -Fq "SCAN body pose TF ready: map -> base_footprint" \
    < <(tail -c "+$((RUN_LOG_OFFSET + 1))" "$LOG_FILE" 2>/dev/null)
}

cleanup() {
  local status=$?
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log_info "收到停止信号，正在停止导航定位链路"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  rm -f "$PID_FILE" "$ADAPTER_PID_FILE" "$READY_FILE" "${CHILD_PID_FILES[@]}"
  write_json_file "$STATUS_FILE" "{\"running\":false,\"scene_dir\":\"$SCENE_DIR\",\"exit_code\":$status}"
  exit "$status"
}

trap cleanup INT TERM EXIT

log_info "启动导航定位链路：$SCENE_DIR"
log_info "map.pcd：$MAP_PCD"
log_info "ground.pcd：$GROUND_PCD"
log_info "Livox 雷达 IP：$LIVOX_LIDAR_IP"
log_info "Livox 配置文件：$LIVOX_CONFIG_PATH"
log_info "导航控制链：SCAN planner=$NAV_ENABLE_SCAN_PLANNER controller=$NAV_ENABLE_SCAN_CONTROLLER path_follower=$NAV_ENABLE_PATH_FOLLOWER"
log_info "安全链：dynamic_avoidance=$NAV_ENABLE_DYNAMIC_AVOIDANCE Navigation底盘直控=$NAV_ENABLE_ROBOT_CONTROL"
log_info "底盘适配：model=$NAV_ROBOT_MODEL cmd_vel=$NAV_ROBOT_CMD_VEL_TOPIC go2_method=$NAV_GO2_CONNECTION_METHOD go2_ip=${NAV_GO2_IP:-未设置}"
if [ -n "$LIVOX_HOST_IP" ]; then
  log_info "本机雷达网卡 IP：$LIVOX_HOST_IP"
else
  log_warn "未设置 LIVOX_HOST_IP，请确认本机网卡与雷达同网段"
fi
if [ -n "$PLANGROUND_PCD" ]; then
  log_info "footprint_fill/fill_footpoint.pcd：$PLANGROUND_PCD"
else
  log_warn "未找到 footprint_fill/fill_footpoint.pcd，将不传入 planground"
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
  "enable_scan_planner:=$NAV_ENABLE_SCAN_PLANNER"
  "enable_scan_controller:=$NAV_ENABLE_SCAN_CONTROLLER"
  "enable_scan_tf_pose:=true"
  "enable_path_follower:=$NAV_ENABLE_PATH_FOLLOWER"
  "enable_dynamic_avoidance:=$NAV_ENABLE_DYNAMIC_AVOIDANCE"
  "enable_waypoint_monitor:=true"
  "enable_robot_control:=$NAV_ENABLE_ROBOT_CONTROL"
  "robot_model:=$NAV_ROBOT_MODEL"
  "robot_cmd_vel_topic:=$NAV_ROBOT_CMD_VEL_TOPIC"
  "robot_max_linear_x:=0.10"
  "robot_max_linear_y:=0.05"
  "robot_max_angular_z:=0.30"
  "go2_connection_method:=$NAV_GO2_CONNECTION_METHOD"
  "go2_ip:=$NAV_GO2_IP"
  "go2_serial_number:=$NAV_GO2_SERIAL_NUMBER"
  "go2_aes_128_key:=$NAV_GO2_AES_128_KEY"
  "go2_ensure_motion_mode:=false"
  "go2_stand_on_connect:=false"
  "go2_stand_command:=BalanceStand"
  "go2_move_command_mode:=no_reply"
  "go2_use_remote_command_from_api:=false"
  "go2_enable_builtin_obstacle_avoidance:=false"
  "rviz:=false"
)

if [ -n "$LIVOX_HOST_IP" ]; then
  launch_args+=("host_ip:=$LIVOX_HOST_IP")
fi

if [ -n "$PLANGROUND_PCD" ]; then
  launch_args+=("planground_pcd:=$PLANGROUND_PCD")
fi

RUN_LOG_OFFSET="$(stat -c '%s' "$LOG_FILE" 2>/dev/null || printf '0')"
ros2 launch nav_bringup navigation.launch.py "${launch_args[@]}" >> "$LOG_FILE" 2>&1 &

LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" > "$PID_FILE"

ready_waited=0
while kill -0 "$LAUNCH_PID" 2>/dev/null && [ "$ready_waited" -lt "$NAV_READY_TIMEOUT_SECONDS" ]; do
  if record_navigation_children && global_planner_map_ready && scan_body_pose_ready; then
    break
  fi
  sleep 1
  ready_waited=$((ready_waited + 1))
done

if kill -0 "$LAUNCH_PID" 2>/dev/null \
  && record_navigation_children \
  && global_planner_map_ready \
  && scan_body_pose_ready; then
  write_json_file "$READY_FILE" "{\"ready\":true,\"run_id\":\"$RUN_ID\",\"scene_dir\":\"$SCENE_DIR\",\"map_pcd\":\"$MAP_PCD\",\"ground_pcd\":\"$GROUND_PCD\",\"planground_pcd\":\"$PLANGROUND_PCD\",\"pid\":$LAUNCH_PID,\"control_chain\":\"scan_planner_cmd_vel_safe\"}"
  write_json_file "$STATUS_FILE" "{\"running\":true,\"scene_dir\":\"$SCENE_DIR\",\"pid\":$LAUNCH_PID,\"stage\":\"running\"}"
  log_info "导航定位进程已启动：PID=$LAUNCH_PID"
  log_info "ready 文件：$READY_FILE"
else
  log_error "导航定位关键节点或全局规划地图未在 ${NAV_READY_TIMEOUT_SECONDS} 秒内就绪，请查看日志：$LOG_FILE"
  exit 1
fi

wait "$LAUNCH_PID"
