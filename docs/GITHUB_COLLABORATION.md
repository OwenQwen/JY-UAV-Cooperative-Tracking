# GitHub结构与多人协作指南

## 1. Git、GitHub和仓库分别是什么

- **Git** 是本地版本数据库。每次 commit 都保存一组可追踪的文件变化和父提交。
- **GitHub** 托管 Git 仓库，并增加 Issues、Pull Requests、代码审查、Actions CI、Releases、权限和安全规则。
- **repository（仓库）** 应围绕一个能够独立理解和版本化的产品边界，而不是整台电脑的文件备份。
- **working tree（工作区）** 是当前磁盘文件；**staging area（暂存区）** 是下一次提交清单；**commit** 是不可变历史节点。
- **branch（分支）** 是指向某个提交的可移动名称；它不是文件夹，也不是用来挂依赖包的目录。
- **tag（标签）** 固定指向发布提交，例如 `v1.0.0`。

## 2. 本项目为什么采用单仓库多ROS包

三个原创ROS包共同完成一个比赛系统，接口修改往往需要同时变更，因此放在同一仓库：

```text
JY-UAV-Cooperative-Tracking
├── infer_track
├── infer_track_interfaces
└── uav_tracking_control
```

这样一次Pull Request可以原子地修改消息定义、视觉发布端和飞控接收端。PX4等拥有独立发布周期和许可证的上游项目继续保持独立仓库。

当某个模块拥有独立团队、独立发布周期或不同访问权限时，再拆成新仓库。例如未来可拆出：

- `jy-uav-vision-models`：训练脚本、数据描述和模型Release；
- `jy-uav-hardware`：CAD、PCB和打印件，使用Git LFS；
- `jy-uav-deployment`：Jetson Docker镜像与设备服务。

## 3. 原创文件与第三方文件如何分区

原创代码放在正常源码目录，第三方依赖只在清单中声明：

```text
原创：infer_track/、infer_track_interfaces/、uav_tracking_control/
依赖：dependencies.repos、simulation.repos、versions.lock.yaml
说明：docs/DEPENDENCIES.md
```

不要把 `PX4-Autopilot/`、`px4_msgs/`、Agent源码复制到主仓库。这样可以清楚回答：谁写的、按什么许可证、升级从哪里来、当前固定在哪个提交。

## 4. 推荐分支模型

小团队使用简单的trunk-based流程：

- `main`：始终可构建、可演示；禁止直接push。
- `feature/<短名称>`：新功能，例如 `feature/target-reacquire`。
- `fix/<短名称>`：缺陷修复，例如 `fix/px4-status-qos`。
- `docs/<短名称>`：文档，例如 `docs/jetson-sop`。
- `experiment/<短名称>`：不保证合并的实验。
- `release/x.y`：只有需要同时维护旧比赛版本时才创建。

不建议为每个人建立永久分支，也不建议默认增加长期 `develop`；它们容易与 `main` 长期分叉。

## 5. 团队成员的日常流程

首次克隆：

```bash
git clone https://github.com/OwenQwen/JY-UAV-Cooperative-Tracking.git
cd JY-UAV-Cooperative-Tracking
git config user.name "Your Name"
git config user.email "your-public-or-noreply-email"
```

开始任务：

```bash
git switch main
git pull --ff-only origin main
git switch -c feature/target-reacquire
```

查看和提交：

```bash
git status
git diff
git add infer_track/infer_track/target_evidence.py
git diff --cached
git commit -m "feat(tracking): add bounded target reacquisition"
git push -u origin feature/target-reacquire
```

然后在GitHub创建Pull Request，说明目的、风险、测试方法和SITL证据。

同步主分支：

```bash
git fetch origin
git rebase origin/main
git push --force-with-lease
```

只允许在自己的功能分支使用 `--force-with-lease`，绝不对 `main` 强推。

## 6. 外部贡献者

团队内有write权限的成员直接从同一仓库建分支。陌生外部贡献者使用Fork：

1. Fork主仓库；
2. 从Fork建立功能分支；
3. push到Fork；
4. 向上游 `main` 提交Pull Request。

Fork是权限边界，branch是同仓库内的开发历史，两者用途不同。

## 7. Commit和Pull Request规范

推荐 Conventional Commits：

```text
feat(control): add waypoint arrival debounce
fix(px4): match vehicle_status_v1 QoS
test(protocol): cover malformed referee frame
docs(jetson): document serial agent startup
chore(deps): pin px4_msgs v1.16.2
```

一个commit只解决一个逻辑问题。不要把格式化全仓库、模型二进制和飞控逻辑修改塞进同一个commit。

Pull Request至少写清：

- 为什么改；
- 改了哪些节点/话题；
- 是否影响解锁、Offboard、RTL或失联保护；
- 运行了哪些测试；
- SITL日志或视频；
- 是否需要同步参数、Docker或依赖提交。

## 8. main保护规则

在 GitHub `Settings → Rules → Rulesets` 中为 `main` 建规则：

- Require a pull request before merging；
- 至少1个approval；飞控安全代码建议2个；
- Dismiss stale approvals；
- Require status checks：`source-and-tests`；
- Require conversation resolution；
- Block force pushes；
- Block deletions；
- 管理员也不要默认绕过。

小团队推荐Squash merge，使一个PR在 `main` 中对应一个清晰提交。发布时创建tag和GitHub Release：

```bash
git switch main
git pull --ff-only
git tag -a v1.0.0 -m "Competition source release v1.0.0"
git push origin v1.0.0
```

## 9. CODEOWNERS

当前由仓库所有者统一审查；成员账号确定后，可按模块继续拆分：

```text
*                              @OwenQwen
/uav_tracking_control/         @OwenQwen
/infer_track/                  @OwenQwen
/infer_track_interfaces/       @OwenQwen
/simulation/                   @OwenQwen
/.github/                      @OwenQwen
```

消息接口应同时要求视觉和飞控负责人审查，因为它影响多个节点。

## 10. 冲突怎么处理

先更新自己的分支：

```bash
git fetch origin
git rebase origin/main
```

Git标出冲突后，逐个编辑文件，再执行：

```bash
git add <resolved-file>
git rebase --continue
```

不理解冲突双方含义时不要随便选择“全部接受当前/传入”，应找对应模块负责人共同确认。飞控参数、消息字段和坐标系冲突尤其不能机械处理。

## 11. 大文件和发布物

Git适合文本源码，不适合不断更新的模型、视频、ROS bag、ULog、固件和CAD二进制：

- 最终模型/固件：GitHub Release assets；
- 需要共同版本化的模型/CAD：Git LFS；
- 大型数据集与长日志：对象存储并在仓库保存元数据、哈希和下载说明；
- 构建结果：CI Artifact，有保留期限，不进入Git。

使用LFS前全员安装：

```bash
git lfs install
git lfs pull
```

## 12. 第一次上传

在GitHub新建**空仓库**，不要在线自动生成README或LICENSE，然后在整理后的本地目录执行：

```bash
git init -b main
git add .
git status
git commit -m "chore: publish post-competition source archive"
git remote add origin https://github.com/OwenQwen/JY-UAV-Cooperative-Tracking.git
git push -u origin main
```

如果远端已经生成了README，应先拉取并合并历史，不要直接强推覆盖：

```bash
git pull --rebase origin main
```

上传后立即启用branch ruleset、Actions、Issues和私密漏洞报告，并创建第一个签名/注释tag。
