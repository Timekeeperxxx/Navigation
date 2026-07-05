# Navigation

这是一个低耦合 ROS2 导航封装工作区。当前阶段先在本地原生 ROS2 环境下开发和调试，稳定后再迁移到 Jetson ARM64，最后再考虑 Docker 化。

## 当前内容

- `src/nav_bringup`：统一启动与配置包。
- `src/nav_runtime`：JSON topic 到兼容脚本的运行时封装。
- `adapters/legacy_scripts`：通用兼容脚本，供上层后端通过 subprocess 调用。
- `tools/test_app`：Navigation 自带轻量测试前后端。
- `docs`：设计、接口、动态避障、覆盖矩阵和实施计划。
- `maps`、`logs`、`runtime`：本地地图、日志和运行时状态目录。

## 构建

```bash
cd /home/frankluo/Projects/Navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行时话题封装

```bash
export ROS_LOG_DIR=/home/frankluo/Projects/Navigation/logs/ros
ros2 launch nav_runtime runtime.launch.py
```

发送建图命令示例：

```bash
ros2 topic pub --once /nav/command_json std_msgs/msg/String "{data: '{\"command\":\"start_mapping\",\"map_dir\":\"/home/frankluo/Projects/Navigation/maps/test_scene\"}'}"
```

订阅结果：

```bash
ros2 topic echo /nav/command_result
ros2 topic echo /nav/runtime_status
```

## 轻量测试前后端

```bash
cd /home/frankluo/Projects/Navigation
bash tools/test_app/run_test_app.sh
```

默认访问：

```text
http://127.0.0.1:8090
```

该测试前后端不依赖 BotDog。页面可以执行建图、停止、地图加载、发布 `initialpose`、发布目标点和取消导航。命令模式支持：

- `auto`：检测到 `nav_runtime` 在线时走 topic，否则走兼容脚本。
- `topic`：只通过 `/nav/command_json`。
- `script`：直接调用 `adapters/legacy_scripts`。

页面点云预览订阅：

```text
/lio/cloud_world
/terrain_map
/nav/local_obstacles
```

点云只做抽样俯视预览，精细检查继续使用 RViz。

## 建图入口

```bash
bash adapters/legacy_scripts/start_mapping.sh /home/frankluo/Projects/Navigation/maps/Scene001
```

底层等价于：

```bash
ros2 launch nav_bringup mapping.launch.py map_dir:=/home/frankluo/Projects/Navigation/maps/Scene001
```

## 导航定位入口

```bash
bash adapters/legacy_scripts/restart_navigation_localization.sh /home/frankluo/Projects/Navigation/maps/Scene001
```

底层等价于：

```bash
ros2 launch nav_bringup navigation.launch.py scene_dir:=/home/frankluo/Projects/Navigation/maps/Scene001
```

## 停止入口

```bash
bash adapters/legacy_scripts/stop_navigation.sh
```

## 当前阶段限制

- 当前已完成 `nav_bringup` 和 `nav_runtime` 的本地构建验证。
- 当前已完成 `tools/test_app` 的本地 HTTP API 和 ROS2 topic 发布验证。
- 真实建图、重定位、规划、动态避障需要在 ROS2 Humble 环境并接入 Livox MID360 后逐项联调。
- `LIVOX_LIDAR_IP` 默认是 `192.168.123.179`，`LIVOX_HOST_IP` 需要按实际本机网卡配置。
