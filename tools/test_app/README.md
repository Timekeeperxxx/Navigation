# Navigation 轻量测试前后端

该工具用于本地验证 Navigation 的公共接口，不依赖 BotDog 代码、数据库或前端组件。

## 启动

```bash
cd /home/frankluo/Projects/Navigation
bash tools/test_app/run_test_app.sh
```

默认地址：

```text
http://127.0.0.1:8090
```

## 功能

- 查看 ROS2 桥接状态、`nav_runtime` 状态、建图状态和导航状态。
- 调用建图兼容脚本或通过 `/nav/command_json` 发送建图命令。
- 列出 `maps/` 下的地图目录。
- 加载地图并启动重定位/规划。
- 发布 `/initialpose`、`/clicked_point`、`goal_yaw`、`/nav_start`、`/nav_stop`。
- 显示 `/global_path` 路径点数和前 20 个路径点。
- 显示点云俯视预览，当前订阅 `/lio/cloud_world`、`/terrain_map`、`/nav/local_obstacles`。
- 显示运行日志尾部。

## 命令模式

页面中的命令模式有三种：

- `auto`：检测到 `nav_runtime` 在线时走 topic，否则直接调用兼容脚本。
- `topic`：只通过 `/nav/command_json` 发送命令，需要先启动 `nav_runtime`。
- `script`：直接调用 `Navigation/adapters/legacy_scripts`。

启动 `nav_runtime`：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/frankluo/Projects/Navigation/logs/ros
ros2 launch nav_runtime runtime.launch.py
```

## 限制

- 点云预览会抽样显示，默认每帧最多 1500 点，适合作为联调预览，不替代 RViz。
- 后端使用 Python 标准库 HTTP 服务，不依赖 FastAPI。
- 真机建图和规划仍需要 Livox、Super-LIO、terrain_analysis 和 global_planner 链路正常。
