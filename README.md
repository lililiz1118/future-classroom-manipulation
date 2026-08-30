# 未来课堂具身智能教学服务机器人

本仓库是东南大学未来技术学院“未来课堂具身智能机器人开发”项目的 ROS 工作空间。项目面向真实教学、实验室与导览场景，目标是把知识问答、移动引导和定点取放组织成可运行、可恢复、可演示的具身智能服务闭环。

## 项目目标与边界

总体方案包含三条业务链路：

1. **知识服务**：语音输入、教学知识库检索增强问答、可信回答与语音播报。
2. **地点引导**：地点指令、任务调度、移动到达、过程提示与到点讲解。
3. **定点取放**：移动至作业区、目标识别与定位、机械臂取放、结果反馈与失败回退。

这些是项目总体目标，不代表所有模块都已在当前分支完成。项目采用限定场景、有限物品类别和渐进集成的路线，优先保证安全、稳定、可复现与可验收。

## 当前分支状态

当前 Git 分支为 `codex/ur3-headless-moveit`，检出目录是 `~/tracer_ws/.worktrees/ur3-headless-moveit`。分支名不是文件夹名；该目录是 Git 为此分支创建的 linked worktree。

本分支已经完成并纳入验证的范围：

- UR3 CB3 的单命令、分阶段、带安全门的 headless 启动；
- UR ROS Driver 2.4.1、真实标定模型、MoveIt `move_group` 与专用 RViz 配置；
- DH Robotics AG95 实体夹爪的启动、串口分包处理、状态检查与关节状态接入；
- Dashboard 状态检查、人工 `START` 确认、低速限制、控制器冲突检查和失败清理；
- 无硬件单元测试、launch 参数检查及相关 Catkin 构建配置。

当前分支**没有实现或接管 Tracer 底盘导航**。`tracer_nav`、定位与移动导航由团队其他成员开发，本分支只保留仓库中的既有代码，并为后续“移动到位后执行机械臂操作”的系统集成提供机械臂侧基础。知识问答、语音交互和总体任务调度同样属于后续集成范围。

## 运行环境

- Ubuntu 20.04 / ROS Noetic
- 工作空间：`/home/jt001/tracer_ws`
- UR3 CB3 控制柜：`192.168.131.3`
- 机器人电脑 UR 专网地址：`192.168.131.1`
- UR ROS Driver：`2.4.1`
- UR 标定哈希：`calib_13945068365021364089`
- MoveIt 规划组：`arm`
- AG95 默认设备：`/dev/dh_gripper_usb`

## UR3 + AG95 启动

先在机器人电脑本地终端执行只读预检。它不会上电、松闸或发送轨迹：

```bash
cd /home/jt001/tracer_ws
./ur3_moveit_headless.sh --preflight-only
```

现场确认工作区无人、独立硬件急停可用后，运行完整入口：

```bash
cd /home/jt001/tracer_ws
./ur3_moveit_headless.sh
```

启动器会显示控制柜状态、标定哈希、夹爪设备和速度限制。只有精确输入大写 `START` 后，才会执行允许的 Dashboard 上电/松闸动作并初始化 AG95。默认速度滑块为 5%，命令行上限为 10%；启动器不会自动解除 Protective Stop、E-Stop、Fault、Violation 或 Recovery，也不会自动执行 MoveIt 轨迹。

RViz 打开后，操作者仍需选择 `arm`、设置目标、点击 `Plan`、目视检查轨迹和真实工作区，最后再手动点击 `Execute`。未加入 Planning Scene 的真实障碍物不会被 MoveIt 检测。

更完整的操作与停止说明见 [`src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md`](src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md)。

## 开发与验证

开发工作树：

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
```

自动化测试必须保持离线，不得连接 UR Dashboard、上电、松闸、改变实体速度或发送轨迹。launch 文件检查使用 `roslaunch --dump-params`，硬件动作只能由现场操作者按安全流程执行。

## 目录说明

- `src/tracer/tracer_ros/tracer_bringup/`：UR3/AG95 headless 启动、launch、配置与测试。
- `src/tracer_nav/`：Tracer 导航相关既有代码；不属于本分支实现范围。
- `src/FAST_LIO_LOCALIZATION/`：定位相关既有代码；不属于本分支实现范围。
- `src/ur_ros/`：Universal Robots ROS Driver 与相关包。
- `src/dh_gripper/`：DH Robotics AG95 驱动与消息。
- `src/chess_robot/`：历史实验代码，不代表当前项目定位；未验证的象棋手眼标定不得用于自主运动。

项目协作与安全约定见 [`AGENTS.md`](AGENTS.md)。
