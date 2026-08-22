# 依赖管理

## 原则

本仓库只维护团队原创代码。第三方项目不复制、不改名成原创目录，也不把它们的完整 Git 历史再次上传。依赖通过上游 URL、分支/标签和精确提交记录。

| 依赖 | 用途 | 归档版本 | 管理方式 |
| --- | --- | --- | --- |
| PX4-Autopilot | 飞控固件与SITL | `6ea3539` (`v1.16.0`) | `simulation.repos`，独立检出 |
| px4_msgs | PX4 ROS 2消息定义 | `392e831` (`v1.16.2`) | `dependencies.repos`，工作区兄弟包 |
| Micro-XRCE-DDS-Agent | PX4与DDS桥接 | `7362281` (`v2.4.3`) | `simulation.repos`，独立构建 |
| PX4 Gazebo models | 官方X500/相机模型 | `e05f431` | PX4子模块，不单独复制 |
| ROS 2 | 中间件与消息 | Jazzy/Humble | 由Ubuntu和ROS发行版决定 |
| PyTorch/Ultralytics | YOLO推理 | 压缩包未保留版本信息 | 按目标GPU单独安装 |

## 为什么使用 `.repos`

ROS社区常用 `vcstool` 清单一次导入多个源码仓库：

```bash
vcs import ~/tracking_ws/src < dependencies.repos
```

清单中的 `version` 可以是分支、标签或提交。公开可复现版本应写精确提交；需要跟踪开发时才使用分支。更新依赖时同时提交：

1. `.repos` 中的新提交；
2. `versions.lock.yaml` 的版本说明；
3. 构建和 `/fmu/out/*` 实际收发验证记录；
4. 不兼容接口的代码修改。

`px4_msgs` 必须匹配 PX4 的消息接口。仅仅 `colcon build` 成功不能证明 ABI/话题兼容，必须观察位置、状态等 PX4 输出话题持续收到有效数据。

## 分支、子模块与复制源码的区别

- **Git分支**：表示同一个项目的不同开发历史，不用于存放第三方依赖。
- **vcstool清单**：最适合多个ROS包组成的工作区，克隆后每个依赖仍是独立仓库。
- **Git submodule**：适合必须固定显示在仓库子目录的依赖，但使用者需要额外执行 `git submodule update --init --recursive`。本项目没有必要把 `px4_msgs` 设成子模块。
- **直接复制/vendor**：只有上游不可获得或需要长期维护补丁时才考虑，同时必须保留许可证和来源。当前不采用。

## PX4补丁

归档里的 PX4 在 Windows 上显示大量 modified，检查后绝大部分只是 Linux 可执行位从 `100755` 变为 `100644`，不是项目修改。官方 `x500_mono_cam_down` 也已存在于锁定的 PX4 Gazebo models 提交中。因此当前没有需要公开的 PX4 fork。

若未来确实修改 PX4：

1. Fork `PX4/PX4-Autopilot`；
2. 从对应 `release/1.16` 或固定标签建立 `feature/...` 分支；
3. 只提交真实补丁，不提交 `build/`；
4. 在本仓库 `simulation.repos` 中将 URL/commit 指向该 fork；
5. 保留 PX4 原许可证与版权。

## Python和Jetson

桌面端可使用 `requirements-vision.txt` 安装 Ultralytics，但 PyTorch 必须先按CUDA/JetPack版本安装。Jetson宿主机和桌面端通常不能共享同一个 Python wheel 或 Docker镜像。

归档没有保留 `pip freeze`，因此没有伪造 PyTorch、OpenCV、Ultralytics 的精确版本。下次在可运行机器上执行并提交锁定结果：

```bash
python3 -m pip freeze > requirements-vision.lock.txt
python3 -c 'import cv2, torch, ultralytics; print(cv2.__version__, torch.__version__, ultralytics.__version__)'
```
