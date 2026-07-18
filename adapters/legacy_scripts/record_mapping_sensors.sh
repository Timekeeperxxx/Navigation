#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

source_navigation_env

OUTPUT_DIR="${1:-$ROBOT_NAV_WS/rosbag2_$(date +%Y_%m_%d-%H_%M_%S)}"
OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
QOS_FILE="$ROBOT_NAV_WS/install/nav_bringup/share/nav_bringup/config/rosbag_livox_qos.yaml"
if [ ! -f "$QOS_FILE" ]; then
  QOS_FILE="$ROBOT_NAV_WS/src/nav_bringup/config/rosbag_livox_qos.yaml"
fi

if [ -e "$OUTPUT_DIR" ]; then
  log_error "录包目录已存在：$OUTPUT_DIR"
  exit 1
fi

lidar_type="$(ros2 topic type /livox/lidar --no-daemon --spin-time 3 2>/dev/null || true)"
imu_type="$(ros2 topic type /livox/imu --no-daemon --spin-time 3 2>/dev/null || true)"
if [ "$lidar_type" != "livox_ros_driver2/msg/CustomMsg" ] ||
   [ "$imu_type" != "sensor_msgs/msg/Imu" ]; then
  log_error "未发现建图传感器话题，拒绝创建空包"
  log_error "/livox/lidar 类型：${lidar_type:-未发现}"
  log_error "/livox/imu 类型：${imu_type:-未发现}"
  exit 1
fi

log_info "开始录制：$OUTPUT_DIR"
log_info "QoS：$QOS_FILE"
log_info "使用 Ctrl+C 正常结束，rosbag2 才会写入 metadata.yaml"

exec ros2 bag record \
  --output "$OUTPUT_DIR" \
  --storage sqlite3 \
  --max-cache-size 67108864 \
  --max-bag-size 4294967296 \
  --qos-profile-overrides-path "$QOS_FILE" \
  /livox/imu /livox/lidar
