# 07 本地开发到 Docker 迁移

## 顺序

本项目不在 Docker 内起步开发。

正确顺序：

1. 本地开发机原生 ROS2 Humble 调通。
2. 本地整理 launch、脚本、配置、地图格式。
3. 本地 BotDog 通过脚本和 topic 调通。
4. 迁移到 Jetson ARM64 原生环境调通。
5. 最后再制作 Docker 镜像。

## 本地开发阶段

目标命令：

```bash
ros2 launch nav_bringup mapping.launch.py ...
ros2 launch nav_bringup navigation.launch.py ...
```

BotDog 仍通过脚本调用：

```bash
bash scripts/start_mapping.sh <map_dir>
bash scripts/restart_navigation_localization.sh <scene_dir>
```

## Jetson 原生阶段

重点验证：

- Ubuntu 22.04
- ROS2 Humble
- ARM64 编译通过
- Livox MID360 网络可达
- Super-LIO 保存 map 成功
- terrain_analysis 保存 ground/footprint 成功
- global_planner 输出 `/global_path`
- BotDog 后端可订阅 topic 和 TF

## Docker 阶段

Docker 只复现已经跑通的原生环境。

运行约束：

```text
--net=host
ROS_DOMAIN_ID 一致
地图目录 volume 挂载
日志目录 volume 挂载
Livox host_ip 与网卡配置正确
```

BotDog 与 Docker 导航的关系：

- 如果 BotDog 在宿主机运行，它可以继续通过 ROS2 topic 访问容器里的节点。
- 如果 BotDog 需要启动/停止导航，不应直接 source 容器环境。
- 脚本应改为调用导航 API 或 `docker exec` wrapper。

第一阶段不实现 Docker API，只保留设计。

