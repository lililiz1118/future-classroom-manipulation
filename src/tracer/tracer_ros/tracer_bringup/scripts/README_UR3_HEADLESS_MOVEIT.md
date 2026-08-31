# UR3 CB3 Headless MoveIt（含 D405）

此入口不使用示教器、External Control URCap、键盘或 MoveIt Servo。一次人工确认后，它通过 Dashboard 上电和松闸、初始化实体 DH AG95，并启动 UR ROS Driver 2.4.1、MoveIt 与 RViz。默认工作流还会独占或复用腕部 D405；不需要启动底盘（base launch）。

## 安全要求

- 工作区和夹爪周围必须无人，独立硬件急停必须真实可用；SSH、Ctrl+C 和软件节点都不能替代急停。
- 只接受 `NORMAL`；`REDUCED` 和其他安全模式都会阻止 READY，命令行不提供绕过开关。
- 脚本绝不自动解除 Protective Stop、E-Stop、Fault、Violation 或 Recovery，也不调用 `restart safety`。
- 启动器不会自动 Execute 任何轨迹；轨迹执行必须由操作者在 RViz 中单独确认。
- 输入 `START` 后 Dashboard 会上电/松闸，且 AG95 可能运动。夹爪驱动不会自动 respawn；异常退出后必须重新运行启动器并再次确认。

## 首次构建或清理后重建

D405 nodelet 必须和机械臂相关包一起构建在当前 worktree 中。不要只依赖主工作区遗留的相机库：

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES='anygrasp_ros;dh_gripper_driver;dh_gripper_msgs;moveit_config;realsense2_camera;tcurdf;tracer_bringup;ur_dashboard_msgs;ur_description;ur_msgs;ur_robot_driver'
source devel/setup.bash
test -f devel/lib/librealsense2_camera.so
```

若缺少该库，启动器会在 Dashboard 上电和松闸之前停止，并提示重新构建。

## 在此工作树启动

在机器人电脑本地终端运行。默认命令无需 base launch，会要求或启动 D405：

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
TRACER_WS="$PWD" ./ur3_moveit_headless.sh
```

启动器显示 `192.168.131.3` 的 robot/safety mode、标定哈希、AG95 设备、D405 状态和目标速度。只有精确输入大写 `START` 才会调用 Dashboard 和初始化夹爪。默认夹爪设备为 `/dev/dh_gripper_usb`，speed slider 为 5%。

只读预检会进行只读网络、ROS 和 Dashboard 检查，并在 `START` 之前退出：不要求确认、不上电、不松闸、不启动相机，也不会改变硬件状态。

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --preflight-only
```

仅运行 UR3、AG95、MoveIt 和 RViz（不要求或启动 D405）时：

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --no-d405
```

需要在已确认的低风险场景中把速度上限设为 10% 时：

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --speed-slider 0.10
```

速度参数只接受大于 0、且不超过 `0.10` 的值。

UR 运行时参数集中在 `config/ur3_runtime.yaml`。当前 generic 内核的受控验证值为
`robot_receive_timeout: 0.10`，启动器还会显式把
`robot_receive_timeout:=0.10` 传给本项目的 launch wrapper；官方
`ur_robot_driver` 源码和官方 `ur_control.launch` 均未修改。

## 控制链健康状态

启动后终端会明确显示 `STARTING`、`READY` 或 `FAULT`：

- `STARTING`：正在收集 robot mode、safety mode、控制程序、轨迹控制器和原始 UR joint states。
- `READY`：robot mode 为 `RUNNING`、safety mode 为 `NORMAL`、`robot_program_running=True`、`ur_arm_scaled_pos_joint_traj_controller` 独占六轴且为 `running`，并已收到至少两帧递增且未超过 0.50 秒的 `/ur/joint_states`。
- `FAULT`：系统曾经 READY，随后上述任一条件失效。首个原因会被锁存，不会因为节点仍存活或信号短暂恢复而重新显示正常。

FAULT 会先停止本启动器管理的 `move_group`，因此不能继续发起新的 MoveIt Execute；
系统不会恢复或继续之前的运动。检查现场安全后，必须完整退出并重新运行控制链。

受控验证故障门禁时，只能在没有轨迹执行且工作区安全的情况下人为停止机器人控制程序。
预期结果是立即出现 `CONTROL_CHAIN_STATE=FAULT`、终端给出首因、`/move_group`
消失且 RViz 无法继续 Execute。该步骤不应在机械臂运动中执行。

## D405 所有权与冲突

默认模式会检查 `/d405/realsense2_camera` 与 `/d405/realsense2_camera_manager`：两者都健康存在时，headless 会复用这个外部 D405；两者都不存在时，headless 会启动自己的 D405。已由 headless 启动的 D405 会随 headless 退出；外部 D405 则会保持运行。

如需先单独启动外部 D405，在一个已 source ROS 环境的终端运行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tracer_bringup ur3_d405_camera.launch
```

然后在另一个终端执行上面的默认命令；它会复用该外部 D405。D455 节点始终是手动停止后才能继续的冲突；在默认 D405 模式中，仅出现部分 D405 节点也是手动停止旧相机启动后才能继续的冲突。启动器从不杀死外部节点。

排查相机节点和流时运行：

```bash
rosnode list | grep -E '^/d(405|455)/'
rostopic hz /d405/color/image_raw
rostopic hz /d405/depth/image_rect_raw
rostopic info /d405/color/image_raw
```

Headless 启动链由唯一的 `/joint_state_aggregator` 汇总 `/ur/joint_states` 和
`/gripper/joint_states`，并以 50 Hz 发布完整 `/joint_states`。原始话题保留用于
硬件诊断；不要再把任一原始话题直接 relay 到 `/joint_states`，否则会重新引入
MoveIt 执行前的关节状态时序竞争。

## RViz 操作

等待终端报告控制链 `READY`、夹爪、D405（默认模式）、速度缩放和 move_group Ready。
`D405 Color` 是可选的 RViz Image 显示，话题为 `/d405/color/image_raw`；AnyGrasp
直接使用 `/d405/depth/color/points`，不依赖该彩色图显示或其 RViz 订阅。

1. 在 MotionPlanning 面板选择 `arm`。
2. 拖动交互标记设置目标。
3. 点击 `Plan`。
4. 目视检查整条动画轨迹及真实工作区。
5. 确认无碰撞风险后才点击 `Execute`。

首次实体测试保持 5%，只规划短距离、低风险轨迹。未加入 Planning Scene 的真实障碍物不会被 MoveIt 检测。

## 停止

在启动终端按 Ctrl+C 后，启动器会先撤销运动执行能力，再停止其管理的 UR Driver、AG95、D405 和 RViz 进程。关闭 RViz 也会结束本次控制链。默认不自动给 UR3 断电，且不会停止复用的外部 D405 或任何其他外部节点。

如果启动器报告已有 `/servo_server`、`/keyboard_jog`、`/move_group`、`/ur/ur_hardware_interface`、`/dh_gripper_driver`、`/gripper_joint_state_relay` 或 `/joint_state_aggregator`，先停止旧控制入口再重试；不要绕过并发检查。
