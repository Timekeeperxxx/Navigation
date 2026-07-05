# 05 BotDog 参考适配方案

## 定位

`BotDog-jetson-main` 只作为现有前后端接入方式的参考案例。Navigation 不能依赖 BotDog 的目录、数据库、FastAPI 代码、脚本路径或运行时文件。

正确关系：

```text
BotDog -> Navigation 公共接口
```

错误关系：

```text
Navigation -> BotDog 内部文件/脚本/数据库
```

因此，本文件只描述如何让 BotDog 作为一个上层应用接入 Navigation，不定义 Navigation 核心架构。

## 当前 BotDog 已有能力

`BotDog-jetson-main` 已经有 ROS2 bridge，不需要前端直接接 ROS2。

关键文件：

- `backend/services_ros_nav.py`
- `backend/services_mapping.py`
- `backend/services_nav_localization.py`
- `scripts/start_mapping.sh`
- `scripts/restart_navigation_localization.sh`
- `docs/ROS2_INTERFACE_CONTRACT.md`

## 当前调用模式

建图：

```text
前端
  -> /api/v1/nav/mapping/set-enabled
  -> services_mapping.py
  -> subprocess 启动 scripts/start_mapping.sh <map_dir>
```

重定位/规划：

```text
前端
  -> /api/v1/nav/localization/restart
  -> services_nav_localization.py
  -> subprocess 启动 scripts/restart_navigation_localization.sh <scene_dir>
```

话题访问：

```text
services_ros_nav.py
  发布 /clicked_point、goal_yaw、/nav_start、/nav_stop、/initialpose、/cmd_vel
  订阅 /global_path、/nav_status、/lio/cloud_world
  查询 TF map -> base_footprint
```

## 推荐适配策略

第一阶段可以不改 BotDog 前端 API，不改业务语义。但这属于 BotDog adapter 目标，不是 Navigation core 目标。

如果需要兼容 BotDog，可只改 BotDog 底层脚本指向：

```text
BotDog scripts/start_mapping.sh
  -> 调用 Navigation 统一 mapping launch

BotDog scripts/restart_navigation_localization.sh
  -> 调用 Navigation 统一 navigation launch
```

如果不想马上修改 BotDog 仓库，可以在 `Navigation/adapters/legacy_scripts` 中实现同名兼容脚本，本地手动替换路径测试。

不要把这些兼容脚本放入 Navigation core launch 内部。

## 必须保持兼容的文件

BotDog 当前依赖以下文件。Navigation core 不应依赖它们；只有 legacy adapter 可以生成这些兼容文件：

```text
data/nav_runtime/current_task.json
data/nav_runtime/navigation_ready.json
data/nav_runtime/*.pid
logs/start_mapping_debug.log
logs/restart_navigation_localization.log
<map_dir>/.ground_generation_started
```

第一阶段为兼容 BotDog 可以继续生成这些机制，但应明确标记为 legacy adapter 输出。

## 建议新增配置项

后续把硬编码路径替换成环境变量：

```text
BOTDOG_ROOT=/home/jetson/Project/BOTDOG/BotDog
ROBOT_NAV_WS=/home/jetson/Navigation
SUPERLIO_WS=/home/jetson/superlio
GLOBAL_PLANNER_WS=/home/jetson/dddmr_navigation_new_local
ROBOT_MAP_ROOT=/home/jetson/Project/BOTDOG/MAPS
LIVOX_LIDAR_IP=192.168.123.179
LIVOX_HOST_IP=192.168.123.xxx
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

本地开发机可以使用不同路径，但脚本逻辑不应写死 `/home/jetson`。

## Docker 之前的边界

在 Docker 之前，BotDog 仍然通过本机脚本和本机 ROS2 workspace 访问导航。

Docker 化之后：

- BotDog 的 rclpy topic 访问仍可保留。
- 脚本调用应改成调用导航容器 API 或 host 上的 wrapper。

但这不是第一阶段工作。
