# SITL运行与接口说明

本页整理归档环境中的比赛操作命令。所有自动控制先在SITL执行；真机步骤需要另行完成飞控参数、遥控接管、地理围栏和无桨台架验证。

## 分终端启动

先构建并加载工作区：

```bash
cd ~/tracking_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to \
  infer_track_interfaces infer_track uav_tracking_control
source install/setup.bash
export MODEL_PATH=/absolute/path/to/authorized-model.pt
```

终端1，PX4 SITL与向下相机X500：

```bash
cd "$PX4_AUTOPILOT_DIR"
make px4_sitl gz_x500_mono_cam_down -d
```

终端2，DDS Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端3，Gazebo图像桥：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'
```

终端4至9，业务节点：

```bash
source ~/tracking_ws/install/setup.bash
ros2 run infer_track sim_recognition_node
ros2 run infer_track udp_yolo
ros2 run infer_track coordinate_transform
ros2 run infer_track tracking_node
ros2 run infer_track referee_udp_node
ros2 run infer_track mission_manager
```

每个 `ros2 run` 应在独立终端执行。最后启动飞行任务节点：

```bash
ros2 run uav_tracking_control mission_node
```

## 任务命令

开始搜索目标1：

```bash
ros2 topic pub --once /navigation_command std_msgs/msg/String \
  "data: '{\"command\": \"start_search\", \"target_id\": 1}'"
```

从目标1区域切换到目标2：

```bash
ros2 topic pub --once /navigation_command std_msgs/msg/String \
  "data: '{\"command\": \"switch_target\", \"observed_target_id\": 1, \"requested_target_id\": 2}'"
```

返航：

```bash
ros2 topic pub --once /navigation_command std_msgs/msg/String \
  "data: '{\"command\": \"return_home\"}'"
```

## 视觉跟踪输入

启用目标1跟踪：

```bash
ros2 topic pub --once /tracking_command std_msgs/msg/String \
  "data: '{\"enable\": true, \"target_id\": 1}'"
```

停止视觉跟踪：

```bash
ros2 topic pub --once /tracking_command std_msgs/msg/String \
  "data: '{\"enable\": false, \"target_id\": 0}'"
```

飞控任务实际接收的实时偏移话题为 `/control_target_offset`，JSON字段包含 `enable`、`target_id`、`offset_x`、`offset_y`、`predicted` 和 `stamp`。偏移单位为米。

## 裁判命令仿真

```bash
ros2 topic pub --once /referee/command \
  infer_track_interfaces/msg/RefereeCommand \
  '{custom_mode: 1, target_system: 1, base_mode: 1}'
```

归档协议含义：

- `custom_mode=1, base_mode=1`：开始静态搜索；
- `custom_mode=1, base_mode=2`：结束静态搜索；
- `custom_mode=2, base_mode=1`：开始移动目标跟踪；
- `custom_mode=2, base_mode=2`：结束移动跟踪并进入返航流程。

## 启动前检查

至少确认：

```bash
ros2 topic list | grep /fmu/
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic echo /fmu/out/vehicle_status_v1 --once
ros2 node list
```

不同PX4消息版本可能带 `_v1` 等后缀，以实际 `ros2 topic list` 和 `config/mission.yaml` 为准。

## 已知限制

- `mission.yaml` 含比赛场地航点，不应直接用于其他场地。
- 模型权重不在仓库中，必须设置 `MODEL_PATH`。
- UDP默认监听 `0.0.0.0`，部署到不可信网络前应通过防火墙或参数限制来源。
- 无RTK基站时只能按普通GNSS精度使用，不能把目标坐标视为厘米级结果。
- `stop_search` 曾出现在手动命令笔记中，但归档代码是否完整实现必须在SITL验证后再使用。
