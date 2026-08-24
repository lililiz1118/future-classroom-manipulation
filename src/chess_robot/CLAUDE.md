# UR3 中国象棋机器人 — 项目文档

## 项目概述

在松灵 Tracer 移动底盘 + UR3 机械臂 + DH AG-160-95 夹爪 (AG95_MB) + D405 腕部相机的平台上，构建**人机对弈**中国象棋机器人：人类走棋后，机器人通过腕部相机识别棋盘变化，由 Pikafish 引擎计算应手，再用机械臂物理执子走棋并吃掉对方棋子。

### 中国象棋 vs 国际象棋的关键差异
| 特性 | 中国象棋 | 国际象棋 |
|------|---------|---------|
| 棋盘 | 9列×10行，棋子在**交叉点**上 | 8×8，棋子在格子里 |
| 棋子 | 扁平圆盘，顶面印汉字 | 立体棋子 |
| 引擎 | Pikafish (Stockfish 派生) | Stockfish |
| Python 库 | 无成熟等价库，需自行封装 | python-chess |
| 通信协议 | UCCI | UCI |
| 特殊规则 | 九宫、楚河汉界、炮翻山 | 王车易位、吃过路兵 |
| 夹取难度 | 扁平棋子需侧夹或推滑 | 立体棋子从上方夹取 |

### 关键设计决策
- **棋盘定位**: 棋盘四角贴 ArUco 标记，D405 腕部相机拍摄后自动计算 3D 位姿
- **走棋检测**: 人类走棋前后各拍一张图，对比 90 个交叉点颜色变化自动识别走法
- **初始摆棋**: 人为摆好开局棋子，机器人不负责摆棋
- **棋子识别**: V1 用游戏状态跟踪（已知各交叉点棋子类型），V2 可选 OCR 识别棋子汉字
- **夹取策略**: ⚠️ 扁平圆盘棋子需特殊处理 — 从上方侧夹、或推滑到指定位置再夹取，需在 Phase 0 实测验证
- **中国象棋引擎**: Pikafish，通过 UCCI 协议通信

## 工作空间

```
本包:  /home/jt001/tracer_ws/src/chess_robot/
SSHFS: /home/yuan/robot_ssh/tracer_ws/src/chess_robot/

其他关键包:
  /home/jt001/tracer_ws/src/my_ur_control/    — 机械臂控制 (urx, gripper, GraspNet)
  /home/jt001/tracer_ws/src/moveit_config/     — MoveIt 配置
  /home/jt001/tracer_ws/src/urdf/tcurdf/       — 机器人整体 URDF
  /home/jt001/tracer_ws/src/camera_ros/        — 相机 ROS 驱动
  /home/jt001/tracer_ws/src/dh_gripper/        — DH 夹爪 ROS 驱动
```

## 架构

```
┌──────────────────────────────────────────────────────────────────┐
│                    Chinese Chess Robot System                    │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐  │
│  │  Perception   │  │ Chess Engine │  │    Motion Planning    │  │
│  │              │  │              │  │                       │  │
│  │ • 棋盘定位    │  │ • 中文象棋规则│  │ • 交叉点→3D坐标映射   │  │
│  │ • 90点检测    │──▶│ • Pikafish   │──▶│ • 扁平棋子取/放轨迹  │  │
│  │ • 变化检测    │  │ • UCCI协议   │  │ • 吃子逻辑            │  │
│  └──────────────┘  └──────────────┘  └───────────┬───────────┘  │
│                                                   │              │
│                                                   ▼              │
│                                           ┌──────────────┐      │
│                                           │   Execution   │      │
│                                           │ • urx movel/j │      │
│                                           │ • DH Gripper  │      │
│                                           └──────────────┘      │
└──────────────────────────────────────────────────────────────────┘
```

## 目标文件结构

```
tracer_ws/src/chess_robot/
├── CLAUDE.md                          # 本文件
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── chess_bringup.launch
├── scripts/
│   ├── chess_main.py                  # 主程序入口 + 游戏状态机
│   ├── chess_engine.py                # 中国象棋规则 + Pikafish UCCI 封装
│   ├── chess_perception.py            # ArUco棋盘检测 + 走棋检测
│   ├── chess_motion.py                # 交叉点→臂基座坐标映射 + 运动原语
│   ├── chess_gripper.py               # DH夹爪扁平棋子夹取策略
│   ├── chess_calibrate_board.py       # 棋盘标定工具
│   ├── verify_hand_eye.py             # 手眼标定验证
│   ├── verify_safe_poses.py           # 安全位姿验证
│   ├── verify_gripper.py              # DH夹爪功能验证 (Phase 0.2)
│   └── test_pick_place.py             # 棋子取放测试
├── config/
│   └── chess_config.yaml              # 棋盘尺寸、ArUco ID、棋子厚度等
└── engine/
    └── pikafish                       # Pikafish 引擎二进制
```

## 现有能力（可复用）

### 机械臂控制
- **urx 库直连** (`my_ur_control/scripts/ur_control/urx/`):
  - `urx.Robot("192.168.131.3")` → 连接 UR 机械臂 (TCP 30002)
  - `robot.movel([x,y,z,rx,ry,rz], acc, vel)` → Cartesian 直线运动，UR 内部做 IK
  - `robot.movej(joints, acc, vel)` → 关节空间运动
  - `robot.getl()` → 获取当前 TCP 位姿 [x,y,z,rx,ry,rz] (旋转矢量格式)
  - `robot.set_freedive(val, timeout)` → freedrive 示教模式
  - `robot.set_tcp(offset)` → 设置 TCP 偏移
- **连接 IP**: `192.168.131.3`

### 夹爪控制
- **DH AG-160-95 夹爪** (AG95_MB, Modbus 协议, 通过 USB 串口):
  - 端口: `/dev/dh_gripper_usb` → `/dev/ttyUSB0`, 波特率: 115200
  - 位置范围: 0 (全闭) ~ 1000 (全开), 速度: 0~1000, 力: 20~100
  - **✅ 推荐主力: 直连串口** (`my_ur_control/scripts/gripper.py`):
    - `AG95NoInit` 类: 跳过初始化直接控制 (避免重复初始化超时)
    - `gripper.set_pos(target_pos)` — 设置目标位置 (0~1000)
    - `gripper.set_vel(velocity)` — 设置速度
    - `gripper.set_force(force)` — 设置力控阈值 (20~100)
    - `gripper.read_pos()` — 读取当前位置
    - `slow_close_until_current(gripper)` — 力控慢闭，检测到电流阈值(800)停止
    - ⚠️ 与 ROS 驱动互斥（共用串口），使用时需先 `kill` ROS driver
  - ⚠️ 备选: ROS 驱动 (`dh_gripper_driver`):
    - 启动: `roslaunch dh_gripper_driver dh_gripper.launch`
    - 话题: `gripper/states`, `gripper/ctrl` (position/speed/force/initialize)
    - 读取可靠性受 USB 口影响，建议实际使用前先用 `verify_gripper.py --direct` 对比
  - 验证脚本: `chess_robot/scripts/verify_gripper.py` (支持 `--direct` / `--ros` 两种模式)

### 相机
- **D405 (腕部)**: 装于 UR 手腕 (`ur_wrist_plate_arm_link`)，眼在手系统
  - 话题: `/camera/d405/color/image_raw`, `/camera/d405/depth/image_rect_raw`
  - 服务: `/get_d405_photo` (camera_control 包)
- ~~**D455 (固定)**~~: 已拆除，不可用

### 坐标变换（可复用函数，在 `ur3_grab.py` 中）
- `transform_target_to_ee(t_cam_ee, q_cam_ee, t_target_cam, q_target_cam)` → 相机系→末端系
- `transform_target_to_base_urx(robot, target_pos, target_quat)` → 末端系→臂基座系
- `compensate_gripper_offset(target_xyz, target_q, gripper_ee_xyz, gripper_ee_q)` → 法兰目标补偿
- `quat2rotvec(quat)` → 四元数→UR旋转矢量

### 手眼标定（来自原项目 `ur3_grab.py`，⚠️ 尚未验证）
```python
# ⚠️ 未验证 — 使用前必须用 freedrive 实际测试
ee_cam_xyz = np.array([-0.009597337799696761, -0.07408851538404479, 0.01670505075735617])
ee_cam_q   = np.array([0.0006344396200247934, -6.58198103146157e-05, 0.006961144575743825, 0.999975567511685])
```

### 安全位姿（✅ 已验证）
```python
# ✅ 已验证安全 — freedrive 验证通过，后续回零操作使用此组数据
home_j = [1.60624647, -2.69281799, 2.24398565, -2.31487161, -1.60637647, 0.00013183]  # 关节角
home   = [0.11292229, -0.07696328, 0.39864631, 0.01683901, -2.58585492, 1.75758600]  # Cartesian 旋转矢量 (FK of home_j)

# 参考位姿 — 已验证但不作为 home（姿态不理想），保留与仿真场景对比
ur_home_pos  = [-0.37137498, 0.03115423, 0.26795007]
ur_home_quat = [0.00833616, 0.68928081, -0.00361613, -0.7244373]
```

## 环境

- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Python**: 3.10 (conda: `global_3.10`)
- **Conda 路径**: `/home/jt001/.conda/envs/global_3.10/bin/python3` (机器人电脑上)
- **SSHFS**: `/home/yuan/robot_ssh/` 映射到 `jt001@192.168.131.1:/home/jt001/`
- **UR 臂 IP**: `192.168.131.3`
- **DH 夹爪**: USB 串口 `/dev/dh_gripper_usb` (Modbus, 115200 baud)

### 必需的 Python 依赖
```
opencv-contrib-python # ArUco 检测
numpy
scipy
```
Pikafish 引擎需单独下载/编译二进制文件。

### 在机器人电脑上运行
所有脚本应在机器人电脑 (jt001@jt001-pc2) 上运行，通过 SSH 或 SSHFS。
UR 臂启动流程见下方"启动流程"。

## 启动流程

### 1. 启动机械臂 UR ROS Driver
```bash
roslaunch tracer_bringup tracer_ur_bringup.launch
```

### 2. 上电并加载外部控制
```bash
rosservice call /ur/ur_hardware_interface/dashboard/power_on "{}"
rosservice call /ur/ur_hardware_interface/dashboard/brake_release "{}"
rosservice call /ur/ur_hardware_interface/dashboard/load_program "filename: 'ext_ctl.urp'"
rosservice call /ur/ur_hardware_interface/dashboard/play "{}"
```

### 3. 启动 DH 夹爪驱动
```bash
roslaunch dh_gripper_driver dh_gripper.launch
```

### 4. 启动相机
```bash
roslaunch realsense2_camera rs_camera_d405.launch
```

### 5. 运行 Chinese Chess 程序
```bash
cd ~/tracer_ws
python3 src/chess_robot/scripts/chess_main.py
```

## 实现阶段

| Phase | 内容 | 依赖 |
|-------|------|------|
| 0. 硬件验证 | 安全位姿、夹爪功能、手眼标定验证、棋子夹取策略测试 | — |
| 1. 基础设施 | 创建 package，安装依赖，配置 Pikafish | Phase 0 |
| 2. 引擎+感知 | `chess_engine.py` (UCCI) + `chess_perception.py` (90点) + 标定 | Phase 1 |
| 3. 运动控制 | `chess_motion.py` (交叉点映射) + `chess_gripper.py` (扁平棋子) + 抓取测试 | Phase 1 |
| 4. 集成 | `chess_main.py` 状态机 + 端到端测试 | Phase 2+3 |
| 5. 完善 | 异常处理、吃子、OCR棋子识别、语音播报 | Phase 4 |

## 关键参数（待实测确认）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `board_cols` | 9 | 棋盘列数 |
| `board_rows` | 10 | 棋盘行数 |
| `board_intersection_spacing` | 0.045m | 相邻交叉点间距（需实测） |
| `board_marker_ids` | [0,1,2,3] | 四角 ArUco ID |
| `pre_grasp_height` | 0.05m | 取子前悬停高度 |
| `piece_thickness` | 0.015m | 棋子厚度（扁平圆盘） |
| `piece_diameter` | 0.035m | 棋子直径 |
| `dh_gripper_position_open` | 1000 | DH夹爪全开位置 (0~1000) ✅ |
| `dh_gripper_position_close` | 0 | DH夹爪全闭位置 |
| `dh_gripper_speed_default` | 200 | 默认开合速度 |
| `dh_gripper_force_grasp` | 80 | 抓取棋子时的力控阈值 (20~100) |
| `dh_gripper_close_for_piece` | 待实测 | 夹棋子时的实际停止位置（取决于棋子厚度） |
| `dh_gripper_current_threshold` | 800 | 力控闭合检测电流阈值（直连串口模式用） |
| `pikafish_think_time` | 1.0s | AI 思考时间 |
| **⚠️ 手眼标定** | — | 来自原项目代码，**尚未验证**，使用前须用freedrive实测 |
| **安全位姿 `home_j` / `home`** | ✅ 已验证 | freedrive 测试安全，**后续回零使用** |
| **参考位姿 `ur_home_pos/quat`** | ✅ 已验证 | 不理想，不作 home；保留参考 |
| **⚠️ 棋子夹取** | — | **需在 Phase 0 实测**扁平圆盘棋子的最优夹取策略 |

## 注意事项

- urx 的 `movel()` 使用**旋转矢量**格式 [rx,ry,rz]，不是四元数也不是欧拉角
- UR 控制器内部执行 IK 和轨迹插值，不需要 MoveIt 发轨迹
- DH 夹爪通过 USB 串口 (`/dev/dh_gripper_usb`) 控制，非网络socket
- ✅ DH 夹爪 ROS 驱动验证通过（Phase 0.2），位置/速度/力控均可用
- ⚠️ 手眼标定来自原项目代码，尚未在当前硬件上验证
- 所有位姿在 arm base 坐标系下 (`ur_arm_base_link`)，不是 `base_link`
- 棋盘需在机械臂 500mm 工作半径内
- 中国象棋棋子是**扁平圆盘**，夹取策略与传统夹取不同，需额外验证
- Pikafish 使用 UCCI 协议（类似 UCI 但针对中国象棋），需自行实现通信封装
