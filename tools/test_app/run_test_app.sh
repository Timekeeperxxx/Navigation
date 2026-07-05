#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${ROBOT_NAV_WS:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-/opt/ros/humble/setup.bash}"
HOST="${NAV_TEST_HOST:-127.0.0.1}"
PORT="${NAV_TEST_PORT:-8090}"

if [ ! -f "$ROS2_SETUP_FILE" ]; then
  printf '[Navigation测试][错误] 找不到 ROS2 环境文件：%s\n' "$ROS2_SETUP_FILE" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ROS2_SETUP_FILE"
set -u

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1090
  source "$WORKSPACE_DIR/install/setup.bash"
  set -u
else
  printf '[Navigation测试][警告] 未找到 install/setup.bash，请先执行 colcon build\n' >&2
fi

export ROBOT_NAV_WS="$WORKSPACE_DIR"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/logs/ros}"
mkdir -p "$ROS_LOG_DIR"

python3 "$SCRIPT_DIR/backend/main.py" \
  --host "$HOST" \
  --port "$PORT" \
  --workspace-dir "$WORKSPACE_DIR"
