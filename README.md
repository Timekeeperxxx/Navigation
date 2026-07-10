# Navigation

这是部署在 Jetson ARM64 + ROS2 Humble 上的机器狗导航工作区，包含 Livox MID360 驱动、Super-LIO 建图/重定位、PCD 全局规划、SCAN-Planner 局部重规划、动态避障和 Go2/B2 底盘适配。

当前开发板工作区：

```text
/home/jetson/Project/BOTDOG/Navigation
```

## 当前内容

- `src/nav_bringup`：统一启动与配置包。
- `src/nav_runtime`：JSON topic 到兼容脚本的运行时封装。
- `src/nav_robot_control`：Go2/B2 可切换底盘控制适配器。
- `src/scan_planner` 及其依赖包：SCAN 局部 B-spline 规划与碰撞检查。
- `adapters/legacy_scripts`：通用兼容脚本，供上层后端通过 subprocess 调用。
- `tools/test_app`：Navigation 自带轻量测试前后端。
- `docs`：设计、接口、动态避障、覆盖矩阵和实施计划。
- `maps`、`logs`、`runtime`：本地地图、日志和运行时状态目录。

## 构建

```bash
cd /home/jetson/Project/BOTDOG/Navigation
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
```

当前工作区共 15 个 ROS2 包，已在开发板完成原生 ARM64 构建。

## 开发板环境

先从模板生成本机配置：

```bash
cp adapters/legacy_scripts/env.example adapters/legacy_scripts/env.local
```

`env.local` 只保存本机路径、网卡和机器人选择，已被 Git 忽略。当前 MID360 默认地址为：

```text
LIVOX_LIDAR_IP=192.168.123.179
LIVOX_HOST_IP=192.168.123.222
```

## 运行时话题封装

```bash
export ROS_LOG_DIR=/home/jetson/Project/BOTDOG/BotDog/logs/ros
ros2 launch nav_runtime runtime.launch.py
```

发送建图命令示例：

```bash
ros2 topic pub --once /nav/command_json std_msgs/msg/String "{data: '{\"command\":\"start_mapping\",\"map_dir\":\"/home/jetson/Project/BOTDOG/MAPS/test_scene\"}'}"
```

订阅结果：

```bash
ros2 topic echo /nav/command_result
ros2 topic echo /nav/runtime_status
```

## 轻量测试前后端

```bash
cd /home/jetson/Project/BOTDOG/Navigation
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
bash adapters/legacy_scripts/start_mapping.sh /home/jetson/Project/BOTDOG/MAPS/Scene001
```

底层等价于：

```bash
ros2 launch nav_bringup mapping.launch.py map_dir:=/home/jetson/Project/BOTDOG/MAPS/Scene001
```

## 导航定位入口

```bash
bash /home/jetson/Project/BOTDOG/BotDog/scripts/restart_navigation_localization.sh \
  /home/jetson/Project/BOTDOG/MAPS/Scene23_多楼层
```

底层等价于：

```bash
ros2 launch nav_bringup navigation.launch.py \
  scene_dir:=/home/jetson/Project/BOTDOG/MAPS/Scene23_多楼层
```

## 停止入口

```bash
bash adapters/legacy_scripts/stop_navigation.sh
```

## Go2/B2 控制切换

默认不启动底盘控制。真机控制时通过参数选择目标：

```bash
ros2 launch nav_bringup navigation.launch.py \
  scene_dir:=/path/to/scene \
  enable_path_follower:=true \
  enable_robot_control:=true \
  robot_model:=go2_webrtc \
  go2_connection_method:=LocalAP
```

```bash
ros2 launch nav_bringup navigation.launch.py \
  scene_dir:=/path/to/scene \
  enable_path_follower:=true \
  enable_robot_control:=true \
  robot_model:=b2 \
  b2_cmd_vel_topic:=/unitree/b2/cmd_vel
```

详细说明见 `docs/17_Go2_B2控制切换.md`。

## SCAN-Planner 局部重规划

已迁入 SCAN-Planner ROS2 包，可在现有 `ground.pcd` + `footprint_fill.pcd` 全局规划基础上启用局部 B-spline 重规划：

```bash
ros2 launch nav_bringup navigation.launch.py \
  scene_dir:=/path/to/scene \
  enable_scan_planner:=true \
  enable_scan_controller:=true \
  enable_dynamic_avoidance:=true
```

BotDog 适配器默认启用 SCAN planner、closed-loop controller 和动态避障。导航数据流为：

```text
/goal_pose
  -> global_planner -> /global_path
  -> scan_initial_path_adapter -> /scan/initial_path
  -> scan_planner_node -> /planning/bspline
  -> closed_loop_controller -> /cmd_vel
  -> dynamic_avoidance_monitor -> /cmd_vel_safe
  -> BotDog loopback ingress -> Unitree adapter -> robot
```

SCAN 的机器狗位姿来自 `map -> base_footprint` TF，由 `scan_tf_pose_publisher` 发布为 `/scan/body_pose`。实时点云输入为 `/lio/cloud_world`；`/lio/odom` 只用作点云射线原点，不再作为 SCAN 机身位置。

ROS 侧只转发安全速度，不再创建第二个 SportClient。详细数据流见 `docs/18_SCAN-Planner集成.md`。

### B2 碰撞模型

当前参数按 [Unitree B2 官方页面](https://www.unitree.com/cn/b2) 公布的站立尺寸 `1.098m x 0.450m x 0.645m` 设置。SCAN 使用双圆柱覆盖水平轮廓：

```text
double_cylinder_radius=0.36m
double_cylinder_offset=0.28m
global_planner.inscribed_radius=0.23m
```

替换前的 Unitree Go2 参数保存在 `src/nav_bringup/config/robot_size_pre_b2_20260710.yaml`，仅用于恢复 Go2 部署。

Jetson 上的 Python PCD 发布器默认拒绝超过 64 MiB 或 400 万点的原始输入，避免大场景在降采样前耗尽内存。超限地图需要先在大内存主机离线体素降采样，再作为场景导航输入。

## 当前阶段限制

- 当前已完成 Jetson ARM64 上全部 15 个 ROS2 包的原生构建验证。
- 当前已完成 `tools/test_app` 的本地 HTTP API 和 ROS2 topic 发布验证。
- 真实建图、重定位、规划、动态避障需要在 ROS2 Humble 环境并接入 Livox MID360 后逐项联调。
- 当前开发板默认 `LIVOX_LIDAR_IP=192.168.123.179`、`LIVOX_HOST_IP=192.168.123.222`。雷达网口未建立物理链路时，真机联调不会就绪。
