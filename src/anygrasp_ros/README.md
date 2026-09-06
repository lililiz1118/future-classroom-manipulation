# AnyGrasp D405 ROS 节点

`anygrasp_ros` 是独立启动的感知节点：读取 `/d405/depth/color/points`，按点云时间戳查询 D405 到 `ur_arm_base_link` 的 TF，在机械臂基座坐标系中裁剪工作空间，同时保留相机坐标系点云供 AnyGrasp 推理。它不启动 MoveIt、UR 控制器或夹爪，也不会发送运动命令。

## 启动

在 UR3 + D405 控制链已经 READY 后，另开终端运行：

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch anygrasp_ros anygrasp_d405.launch
```

主要输出：

- `/anygrasp/best_grasp`：输入相机坐标系下的最佳抓姿；
- `/anygrasp/best_grasp_base`：同一抓姿变换到 `ur_arm_base_link` 后的结果；
- `/anygrasp/workspace_cloud`：机械臂基座坐标系下、ROI 后且 RANSAC 前的点云；
- `/anygrasp/object_cloud`：机械臂基座坐标系下、RANSAC 后且统计离群点过滤（SOR）前的点云；
- `/anygrasp/input_cloud`：SOR 后、实际送入 AnyGrasp 的相机坐标系点云；
- `/anygrasp/grasp_markers`：候选抓姿的 RViz `MarkerArray`。

## AG95 TCP 姿态调试

在 AnyGrasp 感知节点运行后，可另开终端运行：

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch anygrasp_ros best_grasp_tcp.launch
```

这个独立节点只订阅并转换姿态，不包含 MoveIt、IK、UR、轨迹或夹爪控制：

- 输入 `/anygrasp/best_grasp_base` 必须是 `ur_arm_base_link`；其他 frame 会 warning 并被丢弃；
- 输出 `/anygrasp/best_grasp_tcp` 保留相同的抓取中心、时间戳和 frame，只按 `G +X/+Y/+Z -> TCP +Z/+X/+Y` 转换方向；
- `/anygrasp/best_grasp_tcp_markers` 同时绘制 AnyGrasp G（较长）与目标 AG95 TCP（较短）RGB 坐标轴，方便在 RViz 的 `MarkerArray` display 中核对映射。

`gripper_base_link -> ag95_tcp` 的 0.175 m 是机器人 URDF 中物理 TCP 的固定 TF，绝不能在此节点对 `/best_grasp_tcp` 的 position 重复加入偏移。

同一节点还会发布 `/anygrasp/pre_grasp_tcp`。它的 orientation 与
`/anygrasp/best_grasp_tcp` 完全相同，position 使用 TCP 局部 +Z 轴计算：
`p_pre = p_grasp - pregrasp_distance * z_TCP`。默认
`pregrasp_distance` 为 0.08 m，必须是有限正数。`/anygrasp/pre_grasp_markers`
会显示 pre-grasp TCP、grasp TCP，以及从 pre-grasp 指向 grasp 的 approach 箭头。

## MoveIt wrist 目标几何桥接

`tcp_to_wrist_goal.launch` 是独立的只读 TF 几何节点。它把
`/anygrasp/best_grasp_tcp` 和 `/anygrasp/pre_grasp_tcp` 转成当前 MoveIt
endpoint `ur_arm_wrist_3_link` 的 `/anygrasp/grasp_wrist_goal` 和
`/anygrasp/pre_grasp_wrist_goal`。每次均从 TF 查询
`base_link <- ur_arm_base_link` 与 `ur_arm_wrist_3_link <- ag95_tcp`，计算
`T_base_wrist = T_base_urbase @ T_urbase_tcp @ inverse(T_wrist_tcp)`；不包含
任何 AG95、tool 或 TCP 偏移硬编码，也不会发布目标 TF 或执行运动。

若点云时间戳对应的 TF 不可用，节点会跳过该帧，不会退回到相机坐标系 ROI。当前 ROI 位于 `ur_arm_base_link`，边界和话题名统一配置在 `config/anygrasp_d405.yaml`。

模型、点云、ROI 和推理参数位于 `config/anygrasp_d405.yaml`。CPU 资源只在
`config/anygrasp_resources.yaml` 中配置，当前 generic 内核的保守值为：

- PyTorch intra-op：2；inter-op：1；
- `OMP_NUM_THREADS=2`；
- `MKL_NUM_THREADS=2`（当前 PyTorch 构建使用 MKL）；
- `OPENBLAS_NUM_THREADS=2`（当前 NumPy 构建使用 OpenBLAS）；
- nice 增量：10。

nice 是相对进程启动基线增加的值。`robot-lz` 上控制链登录会话应为 nice 0，因而
AnyGrasp 的预期实际值是 nice 10；UR Driver、MoveIt、D405 和 RViz 保持 nice 0。
`/etc/security/limits.d/99-realtime.conf` 中 `priority` 表示 Unix nice，而不是实时
优先级，必须保持为 0，不能写成 99（99 会被截断为最低优先级 nice 19）。

这些值是可调验证参数，不是不可更改的硬编码。可复制 YAML、修改并通过 launch
参数替换，而不改节点脚本：

```bash
roslaunch anygrasp_ros anygrasp_d405.launch \
  resource_config_file:=/absolute/path/to/anygrasp_resources.yaml
```

节点启动日志会输出 OMP/MKL/OpenBLAS 请求值、实际 nice，以及 PyTorch 请求值和
实际值。环境线程限制在导入 NumPy 前应用，PyTorch intra/inter-op 限制在导入
`gsnet` 前应用。

## 桌面平面过滤

节点在 base-frame ROI 之后执行一次 Open3D `segment_plane`。它只检查返回的最大
平面，不迭代寻找其他平面。候选平面必须同时满足：法向与 `ur_arm_base_link` 的
Z 轴夹角不超过配置值（用绝对点积消除法向正负二义性）、inlier Z 中位数位于临时
桌面高度窗口内，以及内点数量和比例达到下限。日志会输出平面模型、候选高度、
内点数/比例和过滤后的点数。

`ransac_table_height_min: 0.20` 与 `ransac_table_height_max: 0.28` 只是初始估计，
不是已确认的桌面高度。应结合日志中的候选高度和 `/anygrasp/workspace_cloud` 实测
结果再调整。若 RANSAC 关闭、ROI 点数不足、Open3D 出错、候选平面不合理，或去
平面后物体点过少，本帧会回退到原 ROI 点云；`/anygrasp/object_cloud` 仍会发布，
AnyGrasp 节点不会因该次桌面检测失败而退出。

## 与 UR3 故障门禁的关系

AnyGrasp 与 UR3 启动器保持进程和 launch 解耦。UR3 控制链一旦进入 FAULT，受管
`move_group` 会被停止，新 Execute 被禁止；AnyGrasp 节点仍可能继续发布感知结果，
但这些结果不代表运动控制链健康，也不得用于继续执行。必须完整重启 UR3 控制链并
重新达到 READY。

## 独立桌面几何

`table_surface_publisher_node.py` 是机械臂基础桌面几何的数据源。它订阅
`/d405/depth/color/points`，把限频后的点云通过 TF 转到
`ur_arm_base_link`，复用 `anygrasp_ros.preprocessing` 中的 ROI 和 RANSAC，
并把有效桌面位姿发布到 `/table_surface_pose`。默认检测频率为 1 Hz；单帧
检测失败时不发布错误结果，也不替换上一次锁存的有效位姿。

该节点不会加载 AnyGrasp 神经网络、checkpoint、CUDA 或 YOLO World。
`/table_surface_pose` 是 MoveIt 桌面碰撞体的统一中立话题；旧的
`/yolo_world/table_surface_pose` 不属于 MoveIt 默认启动链。

默认运行 `ur3_moveit_headless.sh` 会在 D405 和 MoveIt 就绪后启动桌面发布器
与碰撞更新器，并等待 `table_surface` 出现在 Planning Scene 后再启动 RViz。
调试时可传入 `--no-table-collision` 关闭整条桌面链。桌面状态超过 5 秒未更新
会标记为 stale，但已有碰撞体不会因偶发 RANSAC 失败而被删除。
