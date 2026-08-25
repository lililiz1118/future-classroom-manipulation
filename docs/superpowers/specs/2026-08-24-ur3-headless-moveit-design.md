# UR3 CB3 Headless MoveIt 启动设计

日期：2026-08-24  
状态：用户已批准并要求实施

## 目标

在机器人电脑 `jt001-pc2` 上提供一个单命令入口，通过 UR Dashboard 完成人工确认后的上电与松闸，再以 UR ROS Driver 2.4.1 的 headless mode 启动实体 UR3、MoveIt 和 RViz。操作者最终在 RViz MotionPlanning 面板中执行 `Plan`、检查轨迹、再执行 `Execute`。

现有 `ur_keyboard_teleop`、MoveIt Servo 和键盘入口保留，但不由新入口启动，且不得与新控制链并发占用六个 UR 关节。

## 固定环境

- 机器人电脑 SSH/Wi-Fi：`jt001@192.168.43.16`
- 机器人电脑 UR 专网接口：`192.168.131.1/24`
- UR3 CB3 控制柜：`192.168.131.3`
- Dashboard：`192.168.131.3:29999`
- ROS：Ubuntu 20.04 / Noetic
- 工作空间：`/home/jt001/tracer_ws`
- UR ROS Driver：2.4.1
- UR 类型：`ur3`
- 标定文件：`tracer_bringup/config/ur3_calibration.yaml`
- 标定哈希：`calib_13945068365021364089`
- MoveIt 组：`arm`
- 实体控制器：`/ur/ur_arm_scaled_pos_joint_traj_controller`

## 启动流程

1. 预检控制柜 IP、Dashboard TCP、robot mode、safety mode、标定 YAML、ROS 包和 Driver 版本。
2. 只允许 `NORMAL`；只有显式传入 `--allow-reduced` 时允许 `REDUCED`。`PROTECTIVE_STOP`、`ROBOT_EMERGENCY_STOP`、`SYSTEM_EMERGENCY_STOP`、`FAULT`、`VIOLATION`、`RECOVERY` 等状态直接退出。
3. 显示 IP、robot mode、safety mode、标定哈希和即将执行的动作。要求操作者输入一次精确确认词。确认前不得发送任何改变控制柜状态的命令。
4. 按需要执行 `power on`，等待 `POWER_ON`、`IDLE` 或 `RUNNING`；按需要执行 `brake release`，等待 `RUNNING`。等待期间任何不允许的 safety mode 都立即中止。
5. 启动只包含校准模型和 UR Driver 的 launch，明确传入 `robot_ip:=192.168.131.3`、`reverse_ip:=192.168.131.1`、`headless_mode:=true`。
6. 等待两条新鲜 `/joint_states`、`/ur/ur_hardware_interface/robot_program_running=True`、目标 scaled trajectory controller 为 `running`，并确认没有第二个 running 运动控制器占用任一 UR 关节。确认 speed-scaling 消息新鲜且数值有效。
7. 调用 `/ur/ur_hardware_interface/set_speed_slider`。默认值为 `0.05`，命令行只接受 `(0, 0.10]`。
8. 单独启动允许 trajectory execution 的 `move_group`，使用现有 simple controller manager 映射到 `ur/ur_arm_scaled_pos_joint_traj_controller/follow_joint_trajectory`。
9. `move_group` ready 后启动现有 MoveIt RViz 配置。
10. 操作者拖动目标、Plan、目视检查规划轨迹、Execute。

## 组件边界

- `headless_dashboard.py`：Dashboard TCP 协议、状态解析、带安全门的状态等待。它不知道 ROS。
- `headless_startup.py`：标定验证、启动顺序和依赖接口。外部副作用通过 Dashboard/Runtime 接口注入，以便离线测试。
- `ur3_headless_moveit.py`：ROS 环境检查、进程生命周期、ROS topic/service/controller readiness 和 CLI。
- `ur3_headless_driver.launch`：加载校准后的 `/robot_description` 并启动 headless Driver，不启动 MoveIt/Servo/RViz。
- `ur3_moveit_execution.launch`：启动允许实体轨迹执行的 `move_group`，不启动 Driver/Servo/RViz。
- `ur3_moveit_headless.sh`：单命令入口，加载 Noetic 和工作空间环境后执行 Python 启动器。

## 故障与停止

- 不自动调用 `unlock protective stop`、`restart safety`、`close safety popup` 或任何恢复动作。
- 预检、确认、上电、松闸、Driver readiness、速度设置、MoveIt readiness 任一步失败，均不进入下一阶段。
- Driver 启动后发生失败时，按 RViz、move_group、Driver 的逆序发送 SIGINT 并等待退出；不自动断电。
- Ctrl+C 使用相同的逆序清理，不遗留 `move_group` 或 UR Driver。
- 若已发现 `/ur/ur_hardware_interface`、`/move_group`、`/servo_server` 或 `/keyboard_jog` 等冲突节点，新入口拒绝启动。

## 安全边界

- 本工具不是安全等级控制器。人工确认必须代表工作区无人且独立硬件急停可用；没有示教器时，尤其不能把软件终止或 SSH 会话当作急停。
- 自动测试不得连接真实 Dashboard、上电、松闸、改变速度或发送轨迹。
- RViz 的规划场景只保护已建模障碍物；未建模物体不会被 MoveIt 检测。
- 新入口默认 5% speed slider，最高只允许 10%。

## 验证

- 单元测试覆盖 Dashboard 响应解析、安全状态白名单、确认前零副作用、上电/松闸顺序、标定哈希、控制器独占和失败清理。
- `roslaunch --dump-params` 验证校准模型、headless 参数、MoveIt trajectory execution 和 controller action 映射，不启动硬件。
- Catkin 只构建相关包或执行现有允许的选定构建；不通过测试触发实体动作。
- 实机验证由操作者在现场完成：启动后先保持至少 30 秒不 Execute，再用 5% 速度执行一条短距离规划。
