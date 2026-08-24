# Robottracer

ROS Noetic 工作空间，包含 Tracer 移动底盘、UR3 机械臂、Livox MID-360
定位导航、相机以及开发中的中国象棋机器人。

## 机器与网络

- 机器人电脑：`jt001@jt001-pc2`，无线 IP `172.20.10.7`
- UR 控制柜：`192.168.131.3`
- 主工作空间：`/home/jt001/tracer_ws`
- 系统：Ubuntu 20.04 + ROS Noetic

## 构建

```bash
cd /home/jt001/tracer_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

当前导航、定位、MPPI 和 UR 启动相关的选定包可以构建。全工作空间构建仍会在
已有的 DH 夹爪包中失败：驱动编译早于消息头生成，后续需单独修复其 CMake
依赖顺序。

## 启动入口

导航：

```bash
roslaunch tracer_nav nav_all.launch
```

象棋 UR 启动：

```bash
roslaunch tracer_bringup chess_ur_startup.launch
```

该入口会执行 UR Dashboard 的上电、松刹车、加载程序和运行操作。启动前确认
机械臂工作区无人、实体急停可触及。

键盘 MoveIt Servo 当前在 `codex/ur-keyboard-teleop` 分支，测试已通过，等待
决定是否合并到 `workplace`。

## 当前状态

- UR3 实机校准已写入描述和 Driver，校准哈希为
  `calib_13945068365021364089`。
- 导航默认使用融合定位与 MPPI，本地依赖已纳入 Git。
- 象棋手眼标定仍未通过验证，现有记录误差约 323–335 mm，禁止据此执行自主
  机械臂动作。

项目维护约定见 `AGENTS.md`；象棋实验详情见 `src/chess_robot/CLAUDE.md` 和
`src/chess_robot/实现计划.md`。
