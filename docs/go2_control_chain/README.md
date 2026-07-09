# Go2 控制链路真机记录

本文记录当前 Navigation 到 Unitree Go2 的已验证控制链路。结论来自 Go2 AP WiFi 真机测试。

## 已验证结论

可用链路：

```text
Navigation /cmd_vel_safe
  -> nav_robot_control/go2_webrtc_bridge
  -> Unitree WebRTC
  -> BalanceStand
  -> MCF Move no_reply
  -> Go2 运动
```

关键点：

- `StandUp` 只能让 Go2 起身，不等于进入可连续移动状态。
- 已验证能移动的前置动作是 `BalanceStand`。
- 当前 Go2 对 `wireless_controller` 和 `obstacles_avoid_move` 链路不稳定或不可用。
- 当前可用速度链路是 MCF/Sport `Move` 的 no-reply 连续发送。
- 速度大小会受 Go2 `SpeedLevel` 和 bridge 的 `mcf_*_scale` 共同影响。

## 推荐启动参数

单独启动 Go2 控制 bridge：

```bash
cd /home/frankluo/Projects/Navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/frankluo/Projects/Navigation/logs/ros

ros2 launch nav_robot_control robot_control.launch.py \
  enable_robot_control:=true \
  robot_model:=go2_webrtc \
  go2_connection_method:=LocalAP \
  robot_max_linear_x:=0.30 \
  robot_max_linear_y:=0.20 \
  robot_max_angular_z:=0.80 \
  go2_speed_level_on_connect:=true \
  go2_speed_level:=2 \
  go2_mcf_linear_scale:=2.0 \
  go2_mcf_angular_scale:=1.5
```

这些参数对应：

```text
go2_ensure_motion_mode:=false
go2_stand_command:=BalanceStand
go2_move_command_mode:=no_reply
go2_use_remote_command_from_api:=false
go2_continuous_gait_on_connect:=false
```

上述几项已经作为 `nav_robot_control` 的 Go2 默认策略。

## 手动速度测试

低速：

```bash
ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
"{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

较快：

```bash
ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
"{linear: {x: 0.30, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

查看 bridge 状态：

```bash
ros2 topic echo --once /robot_control/status --full-length
```

期望看到：

```json
{
  "status": "commanding",
  "move_command_mode": "no_reply",
  "last_go2_command": {
    "mode": "mcf_no_reply",
    "x": 0.6,
    "source_x": 0.3
  }
}
```

其中 `x` 是实际发给 Go2 的 MCF `Move.x`，`source_x` 是 ROS `/cmd_vel_safe.linear.x`。

## 接完整 Navigation

```bash
ros2 launch nav_bringup navigation.launch.py \
  scene_dir:=/path/to/scene \
  enable_path_follower:=true \
  enable_dynamic_avoidance:=true \
  enable_robot_control:=true \
  robot_model:=go2_webrtc \
  go2_connection_method:=LocalAP \
  robot_max_linear_x:=0.30 \
  robot_max_linear_y:=0.20 \
  robot_max_angular_z:=0.80 \
  go2_speed_level_on_connect:=true \
  go2_speed_level:=2 \
  go2_mcf_linear_scale:=2.0 \
  go2_mcf_angular_scale:=1.5
```

## 停止语义

软停：

- 停止发布 `/cmd_vel_safe` 后，watchdog 会触发零速度软停。
- `/cmd_vel_safe` 发布零速度也会走零速度软停。
- 软停不应破坏下一次速度命令。

硬停：

```bash
ros2 topic pub --once /nav_stop std_msgs/msg/Bool "{data: true}"
```

硬停会发送 Go2 `StopMove`，并进入 `estop` 状态。恢复前需要：

```bash
ros2 topic pub --once /nav_start std_msgs/msg/Bool "{data: true}"
```

## 排查记录

已尝试但未作为最终链路：

- `StandUp + Move`：Go2 起身但不连续移动。
- `wireless_controller`：ROS/bridge 显示 `commanding`，但当前 Go2 不执行。
- `obstacles_avoid_move`：API 授权返回 `0`，但当前 Go2 不执行。
- `ContinuousGait(data=1)`：返回 `0`，但当前 Go2 上会导致已验证链路不动，默认关闭。

如果后续换固件或换机器，这些链路可以重新测试，但当前机器以 `BalanceStand + MCF Move no_reply` 为准。
