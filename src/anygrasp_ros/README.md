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
