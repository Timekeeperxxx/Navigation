# 18 SCAN-Planner 集成

## 目标

把 SCAN-Planner ROS2 的局部重规划能力迁入 Navigation，并复用当前地图产物：

- `ground.pcd`：现有全局规划静态地面。
- `footprint_fill.pcd` / `fill_footpoint.pcd`：现有可通行 footpoint/planground。
- `/global_path`：由 `nav_planner` 基于上述 PCD 生成。
- `/scan/initial_path`：`/global_path` 下采样后的 SCAN 参考路径。
- `/lio/cloud_world`：SCAN 局部占据栅格的实时点云输入。
- `/scan/body_pose`：查询 `map -> base_footprint` TF 后生成的机器狗当前位姿。

## 数据流

```text
ground.pcd + footprint_fill.pcd/fill_footpoint.pcd
  -> nav_pcd_map_publisher
  -> /mapground + /planground
  -> global_planner
  -> /global_path
  -> scan_initial_path_adapter
  -> /scan/initial_path

map -> base_footprint (TF)
  -> scan_tf_pose_publisher
  -> /scan/body_pose

/scan/initial_path + /scan/body_pose + /lio/cloud_world
  -> scan_planner_node
  -> /planning/bspline
  -> closed_loop_controller
  -> /cmd_vel
  -> dynamic_avoidance_monitor
  -> /cmd_vel_safe
```

不要把 `ground.pcd` 直接 remap 到 SCAN 的 `/grid_map/cloud`。SCAN 在这里把 `/grid_map/cloud` 当占据/障碍观测使用，直接喂地面点会把可行走地面误标成障碍。

## 启动

在完整导航链路中启用 SCAN：

```bash
ros2 launch nav_bringup navigation.launch.py \
  scene_dir:=/path/to/scene \
  enable_scan_planner:=true \
  enable_scan_controller:=true \
  enable_dynamic_avoidance:=true \
  enable_robot_control:=true
```

只验证规划层：

```bash
ros2 launch nav_bringup planning.launch.py \
  map_pcd:=/path/to/map.pcd \
  ground_pcd:=/path/to/ground.pcd \
  planground_pcd:=/path/to/footprint_fill.pcd \
  enable_scan_planner:=true \
  enable_scan_controller:=false
```

兼容脚本启用：

```bash
NAV_ENABLE_SCAN_PLANNER=true \
NAV_ENABLE_SCAN_CONTROLLER=true \
bash adapters/legacy_scripts/restart_navigation_localization.sh /path/to/scene
```

## 关键参数

配置文件：`src/nav_bringup/config/scan_planner.yaml`

- `scan_body_pose_topic`：默认 `/scan/body_pose`，位姿来自 `map -> base_footprint` TF。
- `scan_sensor_pose_topic`：默认 `/lio/odom`，只用于点云射线原点，不作为机器狗当前位置。
- `scan_cloud_topic`：默认 `/lio/cloud_world`。
- `scan_global_frame`：默认 `map`。
- `scan_robot_frame`：默认 `base_footprint`。
- `scan_tf_pose_rate`：默认 `30Hz`。
- `scan_raw_path_topic`：默认 `/global_path`。
- `scan_initial_path_topic`：默认 `/scan/initial_path`。
- `grid_map/body_height`：从 ground/planground 路径抬到机身轨迹高度，避免和碰撞圆柱偏移混用。

### Unitree B2 尺寸模型

[Unitree B2 官方页面](https://www.unitree.com/cn/b2) 公布的站立尺寸约为 `1.098m x 0.450m x 0.645m`。SCAN 使用双圆柱近似水平轮廓：

- `grid_map/double_cylinder_radius: 0.25`
- `grid_map/double_cylinder_offset: 0.225`
- `grid_map/double_cylinder_center_offset: -0.45`
- `grid_map/body_height: 0.32`
- `grid_map/obstacles_inflation_z_up: 0.10`
- `grid_map/obstacles_inflation_z_down: 0.33`

以 `map -> base_footprint` 的 XY 原点为基准，两个圆柱中心沿机器人航向分别位于后方约 `0.225m` 和 `0.675m`，半径为 `0.25m`。全局规划的 `inscribed_radius` 保持为 `0.23m`。替换前的 **Unitree Go2** 尺寸参数保存在 `src/nav_bringup/config/robot_size_pre_b2_20260710.yaml`，仅用于恢复 Go2 部署。

## 当前边界

- 静态全局路径仍由 `nav_planner` 负责。
- SCAN 负责沿 `/global_path` 生成局部 B-spline，并基于实时点云避障。
- SCAN 的规划起点、滑窗 body pose 和闭环反馈统一使用 `map -> base_footprint`。
- `/cmd_vel_safe` 仍由 Navigation 的动态避障/安全过滤层输出给机器人控制桥。
