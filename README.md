# JY UAV Cooperative Tracking

智能无人系统应用挑战赛协同追踪项目的赛后源码归档。系统面向四旋翼自主起飞、目标区域搜索、车辆识别与跟踪、目标坐标估计、裁判通信和返航流程，使用 ROS 2 与 PX4 Offboard 接口完成任务级控制。

> **安全声明**：本仓库是比赛研发代码，不是经过适航或功能安全认证的飞控产品。默认先在 SITL 中运行。任何真机测试都必须拆桨完成静态检查，并保留遥控器、QGroundControl、地理围栏、失联和返航保护。不要通过关闭 PX4 解锁检查来强行起飞。

## 仓库内容

```text
.
├── infer_track/                 # 视觉、跟踪、坐标变换、任务管理、裁判UDP桥
├── infer_track_interfaces/      # 裁判通信自定义ROS 2消息
├── uav_tracking_control/        # PX4网关、搜索航线和任务控制
├── simulation/worlds/           # 原创Gazebo场景
├── models/                      # 权重发布说明；二进制权重不进入Git历史
├── docs/                        # 架构、依赖、操作与GitHub协作说明
├── dependencies.repos           # ROS工作区依赖：精确锁定px4_msgs
├── simulation.repos             # PX4和XRCE Agent的可复现实验版本
└── versions.lock.yaml           # 已验证环境版本记录
```

这里保存的是团队原创或明确纳入本项目维护的内容。PX4、`px4_msgs`、Micro XRCE-DDS Agent 和 PX4 ROS 2 Interface Library 均属于上游项目，不复制进本仓库；依赖关系及固定提交见 [依赖说明](docs/DEPENDENCIES.md)。

## 功能模块

- `uav_tracking_control`：连接 `/fmu/in/*` 与 `/fmu/out/*`，执行起飞、航点、搜索、视觉跟踪切换和 RTL。
- `infer_track`：相机/仿真图像 YOLO 推理、UDP 检测接收、目标证据融合、坐标变换、裁判协议和任务管理。
- `infer_track_interfaces`：`RefereeCommand`、`RefereeTx`、`RefereeTxResult` 消息定义。
- `jiangyin_drone.sdf`：比赛场地的 Gazebo 场景源文件。

主要话题接口和手动测试命令见 [运行说明](docs/OPERATIONS.md)。

## 已归档环境

比赛仿真环境记录为：

- Ubuntu 24.04 + ROS 2 Jazzy；
- PX4-Autopilot `v1.16.0`，提交 `6ea3539157ca358c70a515878b77077af7d4611d`；
- `px4_msgs v1.16.2`，提交 `392e831c1f659429ca83902e66820d7094591410`；
- Micro XRCE-DDS Agent `v2.4.3`，提交 `73622810d984349b80bbac0ef55fc0b694d62222`。

板载机历史部署为 Jetson L4T R32.7.3 宿主机上的 Ubuntu 22.04 Docker、ROS 2 Humble 和 Agent v2.4.2。两个配置属于不同运行档案，不要把 Jazzy 与 Humble 的构建产物混用。

## 获取源码与依赖

以下示例假定把本仓库克隆到 ROS 2 工作区的 `src` 中：

```bash
mkdir -p ~/tracking_ws/src
cd ~/tracking_ws/src
git clone https://github.com/OwenQwen/JY-UAV-Cooperative-Tracking.git
cd ~/tracking_ws
vcs import src < src/JY-UAV-Cooperative-Tracking/dependencies.repos
```

`dependencies.repos` 使用精确提交而不是浮动分支，确保 `px4_msgs` 消息定义不会在无人知情时变化。升级 PX4 时，应在独立 Pull Request 中同时更新固件、`px4_msgs`、锁文件和通信验证结果。

安装 ROS 依赖并构建：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/tracking_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to \
  infer_track_interfaces infer_track uav_tracking_control
source install/setup.bash
```

视觉节点还需要与当前平台兼容的 PyTorch 和 Ultralytics：

```bash
# 先按照桌面GPU或JetPack版本安装正确的PyTorch
python3 -m pip install -r \
  src/JY-UAV-Cooperative-Tracking/requirements-vision.txt
export MODEL_PATH=/absolute/path/to/authorized-model.pt
```

Jetson 上不要让普通 `pip install torch` 覆盖 NVIDIA/JetPack 对应版本。权重未随仓库发布，原因见 [模型说明](models/README.md)。

## SITL快速启动

可按清单另外拉取归档时使用的 PX4 和 Agent：

```bash
mkdir -p ~/jy_uav_external
vcs import ~/jy_uav_external < \
  ~/tracking_ws/src/JY-UAV-Cooperative-Tracking/simulation.repos
git -C ~/jy_uav_external/PX4-Autopilot \
  submodule update --init --recursive
```

完成 PX4 和 Agent 的上游安装步骤后：

```bash
source /opt/ros/jazzy/setup.bash
source ~/tracking_ws/install/setup.bash
export PX4_AUTOPILOT_DIR=~/jy_uav_external/PX4-Autopilot
export MODEL_PATH=/absolute/path/to/authorized-model.pt

ros2 launch uav_tracking_control simulation_bringup.launch.py \
  px4_dir:=$PX4_AUTOPILOT_DIR
```

这个 launch 面向 SITL，会启动多个进程。首次复现更建议按照 [运行说明](docs/OPERATIONS.md) 分终端启动，以便定位故障。

## 测试

不依赖真机的核心逻辑测试：

```bash
cd ~/tracking_ws/src/JY-UAV-Cooperative-Tracking
PYTHONPATH=infer_track python3 -m pytest -q \
  infer_track/test/test_mission_logic.py \
  infer_track/test/test_position_filter.py \
  infer_track/test/test_referee_protocol.py \
  infer_track/test/test_referee_udp_node.py \
  infer_track/test/test_tracking_logic.py
```

ROS 2 构建和测试：

```bash
cd ~/tracking_ws
colcon build --symlink-install --packages-up-to \
  infer_track_interfaces infer_track uav_tracking_control
colcon test --packages-select infer_track infer_track_interfaces
colcon test-result --verbose
```

## GitHub协作

小团队建议采用受保护的 `main` 加短生命周期功能分支，不建立长期 `develop`：

```text
main
 ├── feature/visual-reacquire
 ├── fix/px4-qos
 ├── docs/jetson-startup
 └── release/v1.0（仅在确实需要维护旧版本时创建）
```

每项改动通过 Pull Request 合并，至少一人复核飞控/安全相关变更，CI 必须通过。完整命令、提交格式、冲突处理、Fork 与团队权限说明见 [GitHub与多人协作指南](docs/GITHUB_COLLABORATION.md) 和 [贡献指南](CONTRIBUTING.md)。

## 许可证

本仓库采用分目录许可：

- `infer_track`、`infer_track_interfaces`、文档及仓库支持文件：Apache-2.0；
- `uav_tracking_control`：MIT；
- 第三方依赖：各自上游许可证；
- 模型权重：未确认前不随源码授权。

详见 [LICENSE](LICENSE) 与 `LICENSES/`。维护团队为 `JL_tracking`，公开联系邮箱为 `198208059+OwenQwen@users.noreply.github.com`；公开前仍需确认所有贡献者同意相应许可证。
