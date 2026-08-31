# AnyGrasp D405 ROS 节点

`anygrasp_ros` 是独立启动的感知节点：读取 `/d405/depth/color/points`，在相机坐标系内运行 AnyGrasp，并发布最佳抓取、MarkerArray 和可选过滤点云。它不启动 MoveIt、UR 控制器或夹爪，也不会发送运动命令。

## 启动

在 UR3 + D405 控制链已经 READY 后，另开终端运行：

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch anygrasp_ros anygrasp_d405.launch
```

模型、点云、ROI 和推理参数位于 `config/anygrasp_d405.yaml`。CPU 资源只在
`config/anygrasp_resources.yaml` 中配置，当前 generic 内核的保守值为：

- PyTorch intra-op：2；inter-op：1；
- `OMP_NUM_THREADS=2`；
- `MKL_NUM_THREADS=2`（当前 PyTorch 构建使用 MKL）；
- `OPENBLAS_NUM_THREADS=2`（当前 NumPy 构建使用 OpenBLAS）；
- nice 增量：10。

这些值是可调验证参数，不是不可更改的硬编码。可复制 YAML、修改并通过 launch
参数替换，而不改节点脚本：

```bash
roslaunch anygrasp_ros anygrasp_d405.launch \
  resource_config_file:=/absolute/path/to/anygrasp_resources.yaml
```

节点启动日志会输出请求值和实际 PyTorch/OMP/MKL/OpenBLAS/nice 值。环境线程限制
在导入 NumPy 前应用，PyTorch intra/inter-op 限制在导入 `gsnet` 前应用。

## 与 UR3 故障门禁的关系

AnyGrasp 与 UR3 启动器保持进程和 launch 解耦。UR3 控制链一旦进入 FAULT，受管
`move_group` 会被停止，新 Execute 被禁止；AnyGrasp 节点仍可能继续发布感知结果，
但这些结果不代表运动控制链健康，也不得用于继续执行。必须完整重启 UR3 控制链并
重新达到 READY。
