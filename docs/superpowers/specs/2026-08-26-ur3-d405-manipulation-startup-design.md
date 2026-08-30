# UR3 + D405 独立抓取启动设计

## 1. 背景与目标

当前 `ur3_moveit_headless.sh` 能安全启动实体 UR3、AG95、MoveIt 和 RViz，但 D405 仍由 `tracer_robot_base.launch` 间接启动。物体识别抓取不需要 Tracer 底盘、MID-360、IMU 或 D455，因此启动整套底盘既扩大了故障面，也让机械臂感知依赖了不属于当前任务的组件。

本设计让 `ur3_moveit_headless.sh` 默认提供一条完整的独立抓取启动链：UR3 + AG95 + D405 + MoveIt + RViz。启动器可以复用已健康运行的 D405；没有 D405 时自行启动；发现不健康或冲突的相机状态时安全失败。底盘、导航、D455 与全局任务编排不纳入该入口。

## 2. 已确认根因

### 2.1 RViz 无法订阅

手动添加的 Image display 中，topic `/d405/color/image_raw` 后带有一个不可见空格。RViz 报告 `Character [ ] at element [21] is not valid in Graph Resource Name`，同时 `rostopic info` 显示 `Subscribers: None`。删除空格后 Image 状态恢复为 OK，证明 D405 图像数据链本身正常。

持久修复是在专用 RViz 配置中内置 `rviz/Image`，并把 topic 精确写成 `/d405/color/image_raw`，不再要求操作者手工输入。

### 2.2 无关 D455 被启动

ROS 环境中存在重复的 `tracer_bringup` 包：

- `/home/jt001/nav_test_ws/src/tracer_bringup`
- `/home/jt001/tracer_ws/src/tracer/tracer_ros/tracer_bringup`

前者的 `tracer_robot_base.launch` 无条件包含 `rs_camera_d455.launch`，而运行进程的 `ROS_PACKAGE_PATH` 又包含该旧副本。实际 roslaunch 日志证明，在命令带有 `enable_d455:=false` 时仍启动了 D455 manager/nodelet。这个重复包/overlay 问题不应通过修改导航工作区来掩盖；新的独立抓取入口不调用 `tracer_robot_base.launch`，并把任何正在运行的 D455 节点视为冲突。

## 3. 范围

### 3.1 包含

- `ur3_moveit_headless.sh` 默认要求 D405 可用。
- 没有 D405 节点时，由 headless 启动器启动专用 D405 launch。
- 已有健康 D405 时复用，不重复占用设备。
- 检查 D405 彩色流、深度流与时间戳是否持续更新。
- 发现 D455、残缺 D405 节点或无帧 D405 时明确失败。
- RViz 默认显示 `/d405/color/image_raw`。
- 启动器退出时只关闭自己启动的 D405，不关闭外部 D405。
- 提供 `--no-d405`，保留纯机械臂调试能力。
- 更新操作文档，给出不启动底盘的标准流程。

### 3.2 不包含

- 不修改 `nav_test_ws`、导航 launch 或 D455 驱动。
- 不启动 Tracer 底盘、CAN、MID-360 或 IMU。
- 不实现物体检测、位姿估计或抓取策略。
- 不改变已标定的 UR3 运动学参数或 D405 外参。
- 不自动拔插、复位 USB，不自动清理外部 ROS 节点。
- 不自动执行任何机械臂轨迹。

## 4. 操作者接口

### 4.1 默认模式

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
TRACER_WS="$PWD" ./ur3_moveit_headless.sh
```

默认要求 D405。启动器根据 ROS 运行态选择复用或自行启动，最终打开包含 D405 彩色画面的 RViz。

### 4.2 纯机械臂模式

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --no-d405
```

该模式不检查、不启动也不等待 D405，适用于只验证 UR3、AG95 或 MoveIt 的场景。它不允许启动 D455。

### 4.3 只读预检

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --preflight-only
```

预检验证包、可执行文件、校准、网络、夹爪设备及当前 ROS 节点冲突，但不打开相机、不上电、不松闸。若默认 D405 模式下发现 D455 或残缺的 D405 节点，预检失败并给出处理提示；完整的现有 D405 节点只记录为候选复用对象，帧流验证留到正常启动阶段。

## 5. 组件设计

### 5.1 专用 D405 launch

在 `tracer_bringup/launch` 新增 `ur3_d405_camera.launch`。它只包含 `realsense2_camera/launch/rs_camera_d405.launch`，不包含底盘、D455、MID-360、IMU 或拍照服务。

该 wrapper 固定当前已验证接口：

- 彩色图像：`/d405/color/image_raw`
- 彩色相机参数：`/d405/color/camera_info`
- 深度图像：`/d405/depth/image_rect_raw`
- 相机节点：`/d405/realsense2_camera`
- manager：`/d405/realsense2_camera_manager`

底层 D405 launch 的分辨率、帧率、点云及 TF 参数保持当前正式配置，不在本次变更中顺带调优。

### 5.2 配置与 CLI

`StartupConfig` 新增不可变布尔字段 `enable_d405`，默认值为 `True`。CLI 新增 `--no-d405`，构造配置时令 `enable_d405=False`。既有命令无需变化即可获得 D405。

安全确认摘要增加相机模式，明确显示 `D405=required` 或 `D405=disabled`。

### 5.3 相机状态分类

`RosRuntime.assert_no_conflicts()` 在现有 UR、MoveIt、Servo、夹爪冲突检查之外，对相机节点进行分类：

1. 任一 `/d455/...` RealSense manager 或 loader 存在：失败。错误信息要求停止 D455/旧底盘 launch，不自动杀节点。
2. D405 manager 与 loader 都不存在：状态为 `absent`，正常启动阶段由本启动器创建。
3. D405 manager 与 loader 都存在：状态为 `external`，正常启动阶段验证数据流后复用。
4. 只有 manager 或只有 loader：状态为 `partial`，失败。错误信息提示先退出不完整的 D405 launch，避免重复 nodelet 或设备占用。
5. `--no-d405` 模式忽略 D405 的存在，但仍拒绝 D455，避免无关设备轮询干扰唯一的 RealSense。

运行态保存相机分类，但不把外部进程加入 `self.processes`。

### 5.4 启动与就绪顺序

正常启动顺序为：

1. 现有只读 preflight 与节点冲突检查。
2. Dashboard 状态检查和人工 `START` 确认。
3. UR Driver 启动并就绪；若没有现有 `robot_state_publisher`，沿用当前逻辑自动启动唯一实例。
4. AG95 启动并就绪。
5. 若启用 D405：
   - `absent`：启动 `ur3_d405_camera.launch`；
   - `external`：不启动新进程；
   - 两种情况都执行同一套 `wait_d405_ready()`。
6. 设置 UR 低速滑块。
7. 启动并等待 MoveIt。
8. 启动 RViz。
9. 监督所有由本入口拥有的子进程。

相机在 RViz 之前就绪，避免 RViz 启动后长期显示 `No Image received`。

### 5.5 D405 就绪判定

`wait_d405_ready()` 必须在 `state_timeout` 内完成以下检查：

- 从 `/d405/color/image_raw` 收到连续两帧 `sensor_msgs/Image`。
- 从 `/d405/depth/image_rect_raw` 收到连续两帧 `sensor_msgs/Image`。
- 每个流的第二帧时间戳严格晚于第一帧。
- 第二帧相对 ROS 当前时间不超过 1 秒，且时间戳不在未来。
- 从 `/d405/color/camera_info` 收到一条 `sensor_msgs/CameraInfo`。
- 图像宽高为正，彩色图像尺寸与 camera_info 一致。

超时时，错误信息必须区分“节点存在但没有彩色帧”“没有深度帧”“时间戳不更新/过期”和“camera_info 不匹配”。不通过降低判定标准继续启动 MoveIt/RViz。

### 5.6 进程所有权与关闭

- 自行启动的 D405 通过现有 `_launch()` 注册为标签 `d405_camera` 的子进程。
- 复用的外部 D405 不加入子进程表，headless 退出时保持运行。
- 自行启动的 D405 按现有逆序关闭策略接受 SIGINT；只有超时后才升级为 SIGTERM。
- 任一后续阶段失败时，由 `finally` 关闭本入口拥有的 RViz、MoveIt、D405、AG95 和 UR Driver；不自动给 UR3 断电，也不触碰外部 D405。

## 6. RViz 配置

`ur3_headless_moveit.rviz` 在既有 Grid、RobotModel 和 MotionPlanning 之外新增：

```yaml
- Class: rviz/Image
  Enabled: true
  Image Topic: /d405/color/image_raw
  Name: D405 Color
  Queue Size: 2
  Transport Hint: raw
  Unreliable: false
```

topic 必须是精确 YAML 字符串，不带前后空白。`--no-d405` 模式仍加载同一 RViz 配置，Image display 可以显示无图像状态，但不得影响 MoveIt 使用；该模式是操作者显式选择的例外。

## 7. 错误处理

所有相机错误使用 `StartupError` 并给出可执行提示：

- D455 冲突：列出检测到的 D455 节点，提示停止旧底盘/D455 launch。
- D405 部分节点：列出缺失与存在的节点，提示退出旧 D405 launch 后重试。
- 外部 D405 无帧：说明不会启动第二个驱动，提示检查现有 D405 终端和 USB。
- 自启 D405 子进程提前退出：报告退出码和 `d405_camera` 标签。
- 数据流超时或过期：指出具体 topic。

启动器不执行 `rosnode kill`、`rosnode cleanup`、USB reset 或设备拔插等自动恢复动作，避免误伤其他系统。

## 8. 测试策略

所有生产代码变更遵循测试先行。

### 8.1 单元测试

- CLI 默认 `enable_d405=True`，`--no-d405` 设为 False。
- `StartupConfig` 正确携带相机模式。
- 无相机节点分类为 `absent`。
- 完整 D405 节点分类为 `external`。
- 单边 D405 节点与任一 D455 节点导致 `StartupError`。
- 启动顺序为 Driver → AG95 → D405 → speed → MoveIt → RViz。
- `external` 模式不调用相机 launch，但仍等待 D405 就绪。
- `absent` 模式使用 `ur3_d405_camera.launch`。
- 彩色/深度时间戳不前进、过期、缺失或 camera_info 不匹配均失败。
- shutdown 关闭自启 D405；外部 D405 不在关闭列表中。

### 8.2 配置测试

- `ur3_d405_camera.launch` 只生成 D405 节点，不生成 D455、底盘、MID-360 或 IMU 节点。
- RViz 配置包含启用的 `rviz/Image`。
- `Image Topic` 与 `.strip()` 后完全相等，且精确为 `/d405/color/image_raw`。
- 原有 Grid、RobotModel、MotionPlanning 与 `base_link` 固定坐标系保持不变。

### 8.3 构建与无硬件验证

- 运行 tracer_bringup 的全部 Python 单元测试。
- 执行 `catkin_make`。
- 使用 `roslaunch --nodes`/解析测试确认专用 launch 不包含无关节点。
- 运行 `./ur3_moveit_headless.sh --preflight-only`，确认不会改变硬件状态或启动相机。

### 8.4 实机验收

实机验收必须保持急停可达、工作区无人，不自动执行轨迹：

1. 不启动底盘，运行默认 headless 入口。
2. 确认没有 `/d455/...` 节点。
3. 确认 `/d405/color/image_raw` 与 `/d405/depth/image_rect_raw` 稳定约 30 Hz。
4. 确认 RViz 的 D405 Color 状态为 OK 且能看到实时画面。
5. 退出 headless，确认自启 D405 一并退出。
6. 单独启动健康 D405 后再次运行 headless，确认复用且退出时不关闭外部 D405。

## 9. 文档与运维约束

更新 `README_UR3_HEADLESS_MOVEIT.md`：

- 默认流程不再要求先启动 `tracer_robot_base.launch`。
- 明确禁止为独立抓取任务启动 D455。
- 记录默认、自带外部 D405 和 `--no-d405` 三种模式。
- 记录 D455 冲突、D405 部分节点、无帧和 RViz topic 错误的诊断命令。
- 强调 worktree 启动方式 `TRACER_WS="$PWD"`，避免重复包/overlay 解析到其他工作区。

## 10. 成功标准

- 单条 headless 命令在不启动底盘的情况下提供 UR3、AG95、D405、MoveIt 和 RViz。
- 健康外部 D405 被复用；不存在时被安全启动；不健康状态不会触发第二个驱动。
- D455 不会由该入口启动，存在 D455 时启动器明确失败。
- RViz 默认显示 D405 彩色画面，无需手工输入 topic。
- 退出时只关闭本入口拥有的相机进程。
- 全部自动化测试和构建通过，实机验收不执行任何轨迹。
