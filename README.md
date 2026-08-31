# 未来课堂机器人：感知与抓取子系统

本仓库是东南大学未来技术学院“未来课堂具身智能机器人开发”项目的感知与抓取子系统。仓库负责把相机感知、物体识别与定位、UR3 运动规划和 AG95 末端执行组织成可运行、可恢复、可验证的操作链路，并向队长维护的总集成仓库提供 ROS 接口。

## 仓库职责与边界

本仓库负责：

1. 相机、深度图与点云等感知数据接入；
2. 教学物品的检测、识别、位姿估计与置信度输出；
3. UR3 + MoveIt 运动规划、轨迹执行与状态反馈；
4. AG95 抓取、搬运、放置和失败回退；
5. “识别 - 定位 - 规划 - 执行 - 反馈”的定点操作闭环。

本仓库不负责 Tracer 底盘导航、语音交互、RAG 问答或全局任务调度。这些能力由其他子仓库开发，最终由总集成仓库固定版本并统一启动。上述感知与操作职责是本仓库的目标边界，不代表所有模块均已完成；当前优先保证 UR3 与 AG95 的安全控制基础。

## 当前分支状态

当前 Git 分支为 `codex/ur3-headless-moveit`，检出目录是 `~/tracer_ws/.worktrees/ur3-headless-moveit`。分支名不是文件夹名；该目录是 Git 为此分支创建的 linked worktree。

本分支已经完成并纳入验证的范围：

- UR3 CB3 的单命令、分阶段、带安全门的 headless 启动；
- UR ROS Driver 2.4.1、真实标定模型、MoveIt `move_group` 与专用 RViz 配置；
- DH Robotics AG95 实体夹爪的启动、串口分包处理、状态检查与关节状态接入；
- Dashboard 状态检查、人工 `START` 确认、低速限制、控制器冲突检查和失败清理；
- 统一 UR 运行策略、`robot_receive_timeout=0.10` 与锁存的 `STARTING/READY/FAULT` 控制链健康门禁；
- 独立 AnyGrasp D405 感知节点及集中、可覆盖的 CPU 线程与 nice 资源策略；
- 无硬件单元测试、launch 参数检查及相关 Catkin 构建配置。

当前分支**没有实现或接管 Tracer 底盘导航**。`tracer_nav`、定位与移动导航由团队其他成员开发，本分支只保留现有机器人工作空间中的集成副本，并为后续“底盘移动到位后执行机械臂操作”提供机械臂侧基础。

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
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --preflight-only
```

现场确认工作区无人、独立硬件急停可用后，运行完整入口：

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh
```

启动器会显示控制柜状态、标定哈希、夹爪设备和速度限制。只有精确输入大写 `START` 后，才会执行允许的 Dashboard 上电/松闸动作并初始化 AG95。默认速度滑块为 5%，命令行上限为 10%；启动器不会自动解除 Protective Stop、E-Stop、Fault、Violation 或 Recovery，也不会自动执行 MoveIt 轨迹。只有 robot `RUNNING`、safety `NORMAL`、控制程序运行、轨迹控制器运行且原始 UR joint states 新鲜时才进入 READY；READY 后任一条件失效都会锁存 FAULT、停止受管 `move_group` 并要求完整重启。

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
- `src/anygrasp_ros/`：独立 D405 AnyGrasp 感知、集中资源策略与测试。
- `src/tracer_nav/`：Tracer 导航相关既有集成副本；由其他子仓库和队友维护。
- `src/FAST_LIO_LOCALIZATION/`：定位相关既有集成副本；不属于本仓库维护边界。
- `src/ur_ros/`：Universal Robots ROS Driver 与相关包。
- `src/dh_gripper/`：DH Robotics AG95 驱动与消息。
- `src/chess_robot/`：历史实验代码，不代表当前项目定位；未验证的象棋手眼标定不得用于自主运动。

项目协作与安全约定见 [`AGENTS.md`](AGENTS.md)。未来新增物体识别、位姿估计和抓取编排功能时，应建立独立 ROS package，并通过明确的 message、service 或 action 与总集成仓库对接。
