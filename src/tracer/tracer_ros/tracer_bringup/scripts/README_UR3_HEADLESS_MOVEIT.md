# UR3 CB3 Headless MoveIt

该入口不使用示教器、External Control URCap、键盘或 MoveIt Servo。它在一次人工确认后通过 Dashboard 上电和松闸，同时初始化实体 DH AG95；随后由 UR ROS Driver 2.4.1 直接发送控制 URScript，并在手臂和夹爪状态都 Ready 后打开 RViz。

## 安全要求

- 工作区和夹爪周围必须无人，独立硬件急停必须真实可用；SSH、Ctrl+C 和软件节点都不能替代急停。
- 默认只接受 `NORMAL`。只有明确理解 UR 的 Reduced 安全配置时才使用 `--allow-reduced`。
- 脚本绝不自动解除 Protective Stop、E-Stop、Fault、Violation 或 Recovery，也不调用 `restart safety`。
- 启动器不会自动 Execute 任何轨迹；轨迹执行必须由操作者在 RViz 中单独确认。
- 输入 `START` 后 AG95 可能运动。夹爪驱动不会自动 respawn；异常退出后必须重新运行启动器并再次确认。

## 启动

在机器人电脑本地终端运行：

```bash
cd /home/jt001/tracer_ws
./ur3_moveit_headless.sh
```

启动器显示 `192.168.131.3` 的 robot/safety mode、标定哈希、AG95 设备和目标速度。只有精确输入大写 `START` 才会调用 Dashboard 和初始化夹爪。默认夹爪设备为 `/dev/dh_gripper_usb`，speed slider 为 5%。

只读预检（不要求确认，不上电、不松闸）：

```bash
./ur3_moveit_headless.sh --preflight-only
```

显式允许 `REDUCED` 并设为 10%：

```bash
./ur3_moveit_headless.sh --allow-reduced --speed-slider 0.10
```

速度参数只接受大于 0、且不超过 `0.10` 的值。

## RViz 操作

等待终端报告 Driver、控制器、速度缩放和 move_group Ready，RViz 打开后：

1. 在 MotionPlanning 面板选择 `arm`。
2. 拖动交互标记设置目标。
3. 点击 `Plan`。
4. 目视检查整条动画轨迹及真实工作区。
5. 确认无碰撞风险后点击 `Execute`。

首次实体测试保持 5%，只规划短距离、低风险轨迹。未加入 Planning Scene 的真实障碍物不会被 MoveIt 检测。

## 停止

在启动终端按 Ctrl+C，启动器会按 RViz、move_group、AG95 Driver、UR Driver 的逆序停止进程。关闭 RViz 也会结束本次控制链。默认不自动给 UR3 断电。

如果启动器报告已有 `/servo_server`、`/keyboard_jog`、`/move_group`、`/ur/ur_hardware_interface`、`/dh_gripper_driver` 或 `/gripper_joint_state_relay`，先停止旧控制入口再重试；不要绕过并发检查。
