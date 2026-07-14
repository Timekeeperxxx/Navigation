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
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-$SCRIPT_DIR/fastdds_navigation.xml}"
NAV_MAX_RAW_PCD_BYTES="${NAV_MAX_RAW_PCD_BYTES:-67108864}"
NAV_MAX_RAW_PCD_POINTS="${NAV_MAX_RAW_PCD_POINTS:-4000000}"

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

reset_navigation_overlay_env() {
  local clean_path=""
  local entry=""
  local old_ifs="$IFS"

  # BotDog usually runs inside its Python venv and may have sourced legacy ROS
  # workspaces. Native ROS executables must start from the board's Humble base.
  IFS=':'
  for entry in ${PATH:-}; do
    case "$entry" in
      *"/.venv/"*|*"/virtualenv/"*) ;;
      *) clean_path="${clean_path:+$clean_path:}$entry" ;;
    esac
  done
  IFS="$old_ifs"
  export PATH="$clean_path"

  unset VIRTUAL_ENV PYTHONHOME
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH CPATH CPLUS_INCLUDE_PATH
}

source_navigation_env() {
  if [ ! -f "$ROS2_SETUP_FILE" ]; then
    log_error "找不到 ROS2 环境文件：$ROS2_SETUP_FILE"
    exit 1
  fi

  reset_navigation_overlay_env

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
  if [ -f "$NAV_FASTDDS_PROFILE" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$NAV_FASTDDS_PROFILE"
    unset FASTDDS_DEFAULT_PROFILES_FILE
  else
    log_warn "未找到 Navigation FastDDS profile：$NAV_FASTDDS_PROFILE"
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  fi
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
  local tmp_path="${file_path}.tmp.$$"
  mkdir -p "$(dirname "$file_path")"
  printf '%s\n' "$content" > "$tmp_path"
  mv -f "$tmp_path" "$file_path"
}

validate_navigation_pcd_input() {
  local file_path="$1"
  local label="$2"
  local file_size=""
  local point_count=""

  if [ ! -f "$file_path" ]; then
    log_error "$label 文件不存在：$file_path"
    return 1
  fi
  if ! [[ "$NAV_MAX_RAW_PCD_BYTES" =~ ^[0-9]+$ ]] || ! [[ "$NAV_MAX_RAW_PCD_POINTS" =~ ^[0-9]+$ ]]; then
    log_error "PCD 安全上限必须是非负整数：bytes=$NAV_MAX_RAW_PCD_BYTES points=$NAV_MAX_RAW_PCD_POINTS"
    return 1
  fi

  file_size="$(stat -c '%s' "$file_path" 2>/dev/null || printf '0')"
  point_count="$(awk 'toupper($1) == "POINTS" { print $2; exit } toupper($1) == "DATA" { exit }' "$file_path" 2>/dev/null || true)"
  if ! [[ "$point_count" =~ ^[0-9]+$ ]]; then
    log_error "$label PCD header 缺少有效 POINTS：$file_path"
    return 1
  fi

  if [ "$NAV_MAX_RAW_PCD_BYTES" -gt 0 ] && [ "$file_size" -gt "$NAV_MAX_RAW_PCD_BYTES" ]; then
    log_error "$label 超过 Jetson 原始 PCD 安全上限：size=$file_size limit=$NAV_MAX_RAW_PCD_BYTES file=$file_path"
    log_error "请先在大内存主机生成体素降采样 PCD，再替换该场景的导航输入文件"
    return 1
  fi
  if [ "$NAV_MAX_RAW_PCD_POINTS" -gt 0 ] && [ "$point_count" -gt "$NAV_MAX_RAW_PCD_POINTS" ]; then
    log_error "$label 超过 Jetson PCD 点数安全上限：points=$point_count limit=$NAV_MAX_RAW_PCD_POINTS file=$file_path"
    log_error "请先在大内存主机生成体素降采样 PCD，再替换该场景的导航输入文件"
    return 1
  fi

  log_info "$label 容量检查通过：bytes=$file_size points=$point_count"
}

find_scene_pcd_file() {
  local scene_dir="$1"
  local exact_name="$2"
  local fallback_pattern="$3"
  local label="$4"
  local selected=""
  local pattern=""
  local old_ifs="$IFS"

  IFS='|'
  for pattern in $exact_name; do
    selected="$(find "$scene_dir" -maxdepth 1 -type f -iname "$pattern" -print -quit 2>/dev/null || true)"
    if [ -n "$selected" ]; then
      break
    fi
  done
  if [ -z "$selected" ]; then
    for pattern in $fallback_pattern; do
      selected="$(find "$scene_dir" -maxdepth 1 -type f -iname "$pattern" -print 2>/dev/null | sort | tail -n 1 || true)"
      if [ -n "$selected" ]; then
        break
      fi
    done
  fi
  IFS="$old_ifs"

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
  local expected_pattern="${4:-}"
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
    if [ -n "$expected_pattern" ]; then
      local cmdline=""
      cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
      if [ -n "$cmdline" ] && [[ "$cmdline" != *"$expected_pattern"* ]]; then
        log_warn "$label PID 已被其他进程复用，拒绝发送信号：PID=$pid cmd=$cmdline"
        rm -f "$pid_file"
        return 0
      fi
    fi
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

      waited=0
      while kill -0 "$pid" 2>/dev/null && [ "$waited" -lt 5 ]; do
        sleep 1
        waited=$((waited + 1))
      done
      if kill -0 "$pid" 2>/dev/null; then
        log_warn "$label 收到 TERM 后仍未退出，发送 KILL"
        if [ -n "$pgid" ] && [ "$pgid" != "$current_pgid" ]; then
          kill -KILL -- "-$pgid" 2>/dev/null || true
        else
          kill -KILL "$pid" 2>/dev/null || true
        fi
      fi
    fi

    # ros2 launch 可能先于其子节点退出。此时只检查 leader PID 会误以为整条
    # 链路已经停止，遗留 relocation/SCAN 等同 PGID 孤儿进程。无论 leader
    # 是否还存在，都对旧进程组做一次有界收尾。
    if [ -n "$pgid" ] && [ "$pgid" != "$current_pgid" ] \
      && ps -eo pgid= 2>/dev/null | awk -v target="$pgid" '$1 == target { found=1 } END { exit !found }'; then
      log_warn "$label 主进程已退出但进程组仍有残留，发送 TERM：PGID=$pgid"
      kill -TERM -- "-$pgid" 2>/dev/null || true
      local group_waited=0
      while ps -eo pgid= 2>/dev/null | awk -v target="$pgid" '$1 == target { found=1 } END { exit !found }' \
        && [ "$group_waited" -lt 5 ]; do
        sleep 1
        group_waited=$((group_waited + 1))
      done
      if ps -eo pgid= 2>/dev/null | awk -v target="$pgid" '$1 == target { found=1 } END { exit !found }'; then
        log_warn "$label 进程组收到 TERM 后仍有残留，发送 KILL：PGID=$pgid"
        kill -KILL -- "-$pgid" 2>/dev/null || true
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
