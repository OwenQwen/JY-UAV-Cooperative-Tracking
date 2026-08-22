# 原始环境整理记录

## 原始快照

解压后的环境约4.24 GiB、11万余文件，主要由PX4完整Git仓库及子模块、Agent源码和构建依赖、两个ROS 2工作区、构建产物与比赛代码组成。

## 纳入公开仓库

- `tracking_ws/src/infer_track` 的Python源码和行为测试；
- `tracking_ws/src/infer_track_interfaces` 的消息定义；
- `tracking_ws/src/uav_tracking_control` 的源码、配置和launch文件；
- `worlds/jiangyin_drone.sdf`；
- 根据现场命令整理的复现文档。

清理时修复了以下发布问题：

- 删除缓存、重复构建树和Windows `Zone.Identifier`附属文件；
- 删除不存在的 `nanosimulate` console entry；
- 补充真实识别节点与键盘节点的console entry；
- 去除 `/home/owen/...`、`/home/cjx/...` 等个人绝对路径；
- 要求通过 `MODEL_PATH` 显式指定模型；
- 补充缺失的ROS包依赖和许可证元数据；
- launch配置改用包共享目录和可配置PX4路径。
- 归档中的裁判协议测试已覆盖Nano转发、飞行器遥测和MAVLink #999/#1000，
  但对应实现仍停留在早期版本；公开副本已依据这些行为测试补齐实现，
  并通过44项纯Python回归测试。原始归档未被修改。

## 未纳入Git历史

| 内容 | 原因 | 推荐发布方式 |
| --- | --- | --- |
| PX4-Autopilot | 第三方且含完整上游Git历史 | `.repos`固定上游提交 |
| px4_msgs | 第三方ROS接口包 | `dependencies.repos` |
| Micro-XRCE-DDS-Agent | 第三方且build目录巨大 | `.repos`固定上游提交 |
| px4-ros2-interface-lib | 第三方，当前业务包未直接使用 | 文档链接/按需清单 |
| build/install/log | 可再生且包含机器绝对路径 | `.gitignore` |
| `best.pt`、`best(1).pt` | 二进制、版本关系和训练授权不明 | 授权确认后Release/LFS |
| `1.mp4` | 运行媒体，不是源码 | GitHub Release或项目演示链接 |

## 公开前仍需人工确认

1. 确认每位贡献者同意目录对应许可证；
2. 确认比赛规则、裁判协议和场地图是否允许公开；
3. 确认YOLO训练数据和权重的再分发权；
4. 在Ubuntu 24.04/Jazzy机器执行完整SITL回归。

## 已执行验证

- 所有Python文件通过`compileall`语法检查；
- 44项纯Python行为测试通过；
- 当前整理环境是Windows且未安装ROS 2，因此节点构建和SITL由GitHub Actions及
  后续Ubuntu 24.04/Jazzy实机回归完成。
