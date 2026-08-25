# UR3 Dashboard 启动、C161 恢复与关机流程

控制器地址默认为 `192.168.131.3:29999`。如现场地址变化，可临时设置
`UR_ROBOT_IP`，不要在多个脚本中分别修改地址。

## 日常启动

正常情况下直接运行：

```bash
roslaunch tracer_bringup chess_ur_startup.launch
```

启动脚本依次执行上电、松闸、加载 `ext_ctl.urp` 和运行。它检测到任何非
`NORMAL/REDUCED` 安全状态时都会停止，不会自动解除保护停机。

## 只读诊断

```bash
rosrun tracer_bringup ur_dashboard_c161_recover.sh --diagnose
```

该命令只读取 PolyScope 版本、机器人模式、安全模式和程序状态。

## C161 恢复

仅在现场已确认错误码为 C161，并完成以下检查后使用：

1. 编码器姿态与实机姿态一致。
2. 工作区无人、无障碍物，急停可用。
3. 断电期间机械臂没有被推动，且没有其他编码器或安全故障。

执行：

```bash
rosrun tracer_bringup ur_dashboard_c161_recover.sh --recover-c161
```

输入确认词 `C161-VERIFIED` 后，脚本只发送一次
`unlock protective stop`，等待安全状态恢复并复查状态。它不会上电、松闸、
加载程序或运行程序。成功后重新运行正常启动命令。

如果当前不是 `PROTECTIVE_STOP`，或者解锁后未恢复为 `NORMAL/REDUCED`，脚本
会拒绝继续。不要把该恢复脚本加入 roslaunch、自启动服务或示教器程序。

## 正常关机

不要直接切断控制柜总电源。运行：

```bash
rosrun tracer_bringup ur_dashboard_shutdown.sh
```

输入确认词 `SHUTDOWN`。脚本会在需要时先停止当前程序，再发送 Dashboard
`shutdown`。等待控制器完全关闭后才能切断总电源。

## 需要停止使用并检修的情况

- 正常关机且断电期间未移动，C161 仍连续复现；
- 编码器角度与实机姿态不一致或读数跳变；
- 同时出现 C74/C75/C76/C77/C78、C160、C216、FAULT 或 VIOLATION；
- 解锁后立即再次保护停机、机械臂下坠、异响或异常动作。

日志文件：

- 启动：`~/.ros/ur_dashboard_startup.log`
- C161 恢复：`~/.ros/ur_dashboard_c161_recovery.log`
- 正常关机：`~/.ros/ur_dashboard_shutdown.log`

根目录的 `temp.sh` 仅作为旧命令兼容入口；正式维护请使用 `scripts/` 中的脚本。
