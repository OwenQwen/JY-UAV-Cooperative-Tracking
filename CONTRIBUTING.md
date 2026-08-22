# Contributing

感谢参与协同追踪项目。任何会影响解锁、模式切换、轨迹设定点、失联保护、RTL、坐标系或裁判通信的改动，都必须在SITL验证并由另一位成员审查。

## 流程

1. 从最新 `main` 创建 `feature/*`、`fix/*`、`docs/*` 或 `test/*` 分支。
2. 保持提交小而明确，使用 `type(scope): summary` 格式。
3. 运行相关Python测试和ROS 2构建。
4. 提交Pull Request并填写模板。
5. 解决CI、审查意见和冲突后再合并。

## 最低验证

```bash
python3 -m compileall -q infer_track uav_tracking_control
PYTHONPATH=infer_track python3 -m pytest -q \
  infer_track/test/test_mission_logic.py \
  infer_track/test/test_position_filter.py \
  infer_track/test/test_referee_protocol.py \
  infer_track/test/test_referee_udp_node.py \
  infer_track/test/test_tracking_logic.py
```

涉及ROS/PX4时还应运行 `colcon build`、启动SITL，并附上关键话题、状态机结果或日志。

## 禁止提交

- 密码、Token、SSH密钥、真实比赛网络凭据；
- `build/`、`install/`、`log/`、缓存和本机绝对路径；
- 未确认授权的数据集、模型权重、地图或裁判程序；
- 为了通过测试而关闭安全检查的代码；
- 未说明来源和许可证的第三方源码。

提交即表示贡献者有权提供该内容，并同意其按目标目录的许可证发布。
