#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_ENV_FILE="${NAV_ENV_FILE:-$SCRIPT_DIR/env.local}"

if [ -f "$NAV_ENV_FILE" ]; then
  set -a
  # shellcheck disable=SC1090
  source "$NAV_ENV_FILE"
  set +a
fi

ROBOT_NAV_WS="${ROBOT_NAV_WS:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ROBOT_NAV_MAP_ROOT="${ROBOT_NAV_MAP_ROOT:-$ROBOT_NAV_WS/maps}"
ROBOT_NAV_LOG_ROOT="${ROBOT_NAV_LOG_ROOT:-$ROBOT_NAV_WS/logs}"
ROBOT_NAV_RUNTIME_ROOT="${ROBOT_NAV_RUNTIME_ROOT:-$ROBOT_NAV_WS/runtime}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$ROBOT_NAV_LOG_ROOT/ros}"
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
LIVOX_LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.123.179}"
LIVOX_HOST_IP="${LIVOX_HOST_IP:-}"
LIVOX_DEFAULT_HOST_IP="${LIVOX_DEFAULT_HOST_IP:-192.168.123.222}"
LIVOX_CONFIG_PATH="${LIVOX_CONFIG_PATH:-$ROBOT_NAV_RUNTIME_ROOT/livox_mid360_config.json}"
SUPERLIO_ROOT_DIR="${SUPERLIO_ROOT_DIR:-$ROBOT_NAV_WS/src/nav_lio}"
NAV_EXTRA_SETUP_FILES="${NAV_EXTRA_SETUP_FILES:-}"

mkdir -p "$ROBOT_NAV_MAP_ROOT" "$ROBOT_NAV_LOG_ROOT" "$ROBOT_NAV_RUNTIME_ROOT" "$ROS_LOG_DIR"

log_info() {
  printf '[Navigation] %s\n' "$*"
}

log_warn() {
  printf '[Navigation][警告] %s\n' "$*" >&2
}

log_error() {
  printf '[Navigation][错误] %s\n' "$*" >&2
}

source_navigation_env() {
  if [ ! -f "$ROS2_SETUP_FILE" ]; then
    log_error "找不到 ROS2 环境文件：$ROS2_SETUP_FILE"
    exit 1
  fi

  set +u
  # shellcheck disable=SC1090
  source "$ROS2_SETUP_FILE"
  set -u

  if [ -f "$ROBOT_NAV_WS/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$ROBOT_NAV_WS/install/setup.bash"
    set -u
  else
    log_warn "未找到 Navigation install/setup.bash，若尚未 colcon build，请先构建工作区"
  fi

  local setup_file
  local old_ifs="$IFS"
  IFS=':'
  for setup_file in $NAV_EXTRA_SETUP_FILES; do
    if [ -z "$setup_file" ]; then
      continue
    fi
    if [ -f "$setup_file" ]; then
      set +u
      # shellcheck disable=SC1090
      source "$setup_file"
      set -u
      log_info "已加载外部 ROS2 工作区：$setup_file"
    fi
  done
  IFS="$old_ifs"

  export ROS_DOMAIN_ID
  export RMW_IMPLEMENTATION
  export ROS_LOG_DIR
}

detect_livox_host_ip() {
  if [ -n "$LIVOX_HOST_IP" ]; then
    printf '%s\n' "$LIVOX_HOST_IP"
    return 0
  fi

  if command -v ip >/dev/null 2>&1; then
    ip -4 route get "$LIVOX_LIDAR_IP" 2>/dev/null \
      | awk '{for (i = 1; i <= NF; i++) if ($i == "src") {print $(i + 1); exit}}'
  fi
}

prepare_livox_config() {
  local detected_host_ip
  detected_host_ip="$(detect_livox_host_ip || true)"

  if [ -n "$detected_host_ip" ]; then
    LIVOX_HOST_IP="$detected_host_ip"
  else
    LIVOX_HOST_IP="$LIVOX_DEFAULT_HOST_IP"
    log_warn "无法自动识别本机雷达网卡 IP，已使用兼容默认值：$LIVOX_HOST_IP；如无点云，请设置 LIVOX_HOST_IP"
  fi

  mkdir -p "$(dirname "$LIVOX_CONFIG_PATH")"
  cat > "$LIVOX_CONFIG_PATH" <<EOF
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "$LIVOX_HOST_IP",
      "cmd_data_port": 56101,
      "push_msg_ip": "$LIVOX_HOST_IP",
      "push_msg_port": 56201,
      "point_data_ip": "$LIVOX_HOST_IP",
      "point_data_port": 56301,
      "imu_data_ip": "$LIVOX_HOST_IP",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "$LIVOX_LIDAR_IP",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
EOF
  export LIVOX_HOST_IP
  export LIVOX_CONFIG_PATH
}

write_json_file() {
  local file_path="$1"
  local content="$2"
  mkdir -p "$(dirname "$file_path")"
  printf '%s\n' "$content" > "$file_path"
}

find_scene_pcd_file() {
  local scene_dir="$1"
  local exact_name="$2"
  local fallback_pattern="$3"
  local label="$4"
  local selected=""

  selected="$(find "$scene_dir" -maxdepth 1 -type f -iname "$exact_name" -print -quit 2>/dev/null || true)"
  if [ -z "$selected" ]; then
    selected="$(find "$scene_dir" -maxdepth 1 -type f -iname "$fallback_pattern" -print 2>/dev/null | sort | tail -n 1 || true)"
  fi

  if [ -z "$selected" ]; then
    log_error "场景缺少 $label：$scene_dir"
    return 1
  fi

  printf '%s\n' "$selected"
}

super_lio_relative_map_dir() {
  local map_file="$1"
  local map_dir
  map_dir="$(dirname "$map_file")"

  if [ -d "$SUPERLIO_ROOT_DIR" ] && command -v realpath >/dev/null 2>&1; then
    realpath --relative-to="$SUPERLIO_ROOT_DIR" "$map_dir" 2>/dev/null || printf '%s\n' "$map_dir"
  else
    printf '%s\n' "$map_dir"
  fi
}

stop_pid_file() {
  local pid_file="$1"
  local label="$2"
  local grace_seconds="${3:-2}"
  local pid=""
  local pgid=""
  local current_pgid=""

  if [ ! -f "$pid_file" ]; then
    return 0
  fi

  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [ -z "$pid" ]; then
    rm -f "$pid_file"
    return 0
  fi

  if kill -0 "$pid" 2>/dev/null; then
    log_info "停止 $label：PID=$pid"
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
    current_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d '[:space:]' || true)"
    if [ -n "$pgid" ] && [ "$pgid" != "$current_pgid" ]; then
      log_info "向 $label 进程组发送 INT：PGID=$pgid"
      kill -INT -- "-$pgid" 2>/dev/null || true
    else
      kill -INT "$pid" 2>/dev/null || true
    fi

    local waited=0
    while kill -0 "$pid" 2>/dev/null && [ "$waited" -lt "$grace_seconds" ]; do
      sleep 1
      waited=$((waited + 1))
    done
    if kill -0 "$pid" 2>/dev/null; then
      log_warn "$label 在 ${grace_seconds} 秒内未退出，发送 TERM"
      if [ -n "$pgid" ] && [ "$pgid" != "$current_pgid" ]; then
        kill -TERM -- "-$pgid" 2>/dev/null || true
      else
        kill -TERM "$pid" 2>/dev/null || true
      fi
    fi
  fi

  rm -f "$pid_file"
}

request_save_terrain_map() {
  local log_file="${1:-/dev/null}"

  if ! command -v ros2 >/dev/null 2>&1; then
    log_warn "当前环境找不到 ros2 命令，跳过 terrain_map 保存请求"
    return 1
  fi

  log_info "请求保存 terrain_map、ground 和 footprint 点云"
  if timeout 45s ros2 service call /save_terrain_map std_srvs/srv/Trigger "{}" >> "$log_file" 2>&1; then
    log_info "terrain_map 保存服务调用完成"
    return 0
  fi

  log_warn "terrain_map 保存服务调用失败或超时，请查看日志：$log_file"
  return 1
}
