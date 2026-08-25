# 快速启动指南

## 系统概述

本系统实现了**轮式里程计 + 点云配准**的融合定位方案，具有以下特点：
- 使用独立的`wheel_odom_frame`坐标系，不与FAST-LIO冲突
- 初始定位后仅配准5次，之后切换到纯轮式里程计模式
- 平滑插值滤波，消除定位抖动

## 启动步骤

### 步骤 1: 编译工作空间
```bash
cd ~/tracer_ws
catkin_make
source devel/setup.bash
```

### 步骤 2: 准备地图文件
确保有PCD格式的全局地图，修改launch文件中的路径：
```bash
# 编辑launch文件
nano src/FAST_LIO_LOCALIZATION/launch/wheel_odom_localization.launch

# 修改这一行
<arg name="map_file" default="/path/to/your/map.pcd" />
```

### 步骤 3: 启动定位系统
```bash
roslaunch fast_lio_localization wheel_odom_localization.launch
```

这将启动：
- ✅ Tracer底盘驱动（发布轮式里程计）
- ✅ Livox MID-360 LiDAR驱动
- ✅ FAST-LIO节点（点云预处理）
- ✅ 地图加载节点
- ✅ 轮式里程计定位节点
- ✅ RViz可视化

### 步骤 4: 设置初始位姿
在RViz中：
1. 点击工具栏的"2D Pose Estimate"按钮
2. 在地图上点击并拖动，设置机器人的初始位置和朝向
3. 等待定位节点输出"Initialization successful!"

### 步骤 5: 验证定位效果
```bash
# 终端1: 查看定位结果
rostopic echo /localization

# 终端2: 查看TF变换
rosrun tf tf_echo map base_link

# 终端3: 查看配准状态
rostopic echo /map_to_odom
```

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                     定位系统架构                          │
└─────────────────────────────────────────────────────────┘

 输入源:
 ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
 │  底盘编码器   │    │ Livox LiDAR  │    │  全局地图     │
 │ /wheel_odom  │    │/cloud_registr│    │  /pcd_map    │
 └──────┬───────┘    └──────┬───────┘    └──────┬───────┘
        │                   │                    │
        │                   │                    │
        └───────────────────┼────────────────────┘
                            │
                    ┌───────▼────────┐
                    │  定位融合节点   │
                    │  (Python 50Hz) │
                    └───────┬────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌───────────────┐  ┌────────────────┐  ┌──────────────┐
│ /localization │  │/odom_to_baselink│  │ TF: map→     │
│ (map→base)    │  │(wheel_odom→base)│  │wheel_odom_frm│
└───────────────┘  └────────────────┘  └──────────────┘
```

## 关键话题

### 输入话题
| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `/wheel_odom` | Odometry | 50Hz | 轮式里程计 |
| `/cloud_registered` | PointCloud2 | 10Hz | FAST-LIO点云 |
| `/pcd_map` | PointCloud2 | 一次 | 全局地图 |
| `/initialpose` | PoseWithCovarianceStamped | 手动 | 初始位姿 |

### 输出话题
| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `/localization` | Odometry | 50Hz | 全局定位结果 |
| `/odom_to_baselink` | Odometry | 50Hz | 轮式里程计输出 |
| `/map_to_odom` | Odometry | 0.5Hz | 地图到里程计变换 |
| `/cur_scan_in_map` | PointCloud2 | 0.5Hz | 可视化用 |
| `/submap` | PointCloud2 | 0.5Hz | 可视化用 |

## 坐标系关系

```
map (固定全局坐标系)
 │
 ├─ odom (FAST-LIO发布，仅用于FAST-LIO内部)
 │   └─ body (LiDAR中心)
 │
 └─ wheel_odom_frame (轮式里程计坐标系)
     └─ base_link (底盘中心)
         └─ body (LiDAR，静态偏移-0.255m)
```

## 参数调优

### 定位精度 vs 平滑度
编辑 `scripts/wheel_odom_localization.py`:

```python
# 更平滑但响应慢
SMOOTH_ALPHA = 0.01  # 降低此值

# 更快响应但可能抖动
SMOOTH_ALPHA = 0.05  # 提高此值
```

### 配准次数
编辑 launch 文件:
```xml
<!-- 更多配准次数，更准确但CPU占用高 -->
<param name="max_registration_count" value="10" />

<!-- 更少配准次数，快速切换到纯里程计 -->
<param name="max_registration_count" value="3" />
```

### 定位频率
```xml
<!-- 更高频率，更准确但CPU占用高 -->
<param name="localization_frequency" value="1.0" />

<!-- 更低频率，节省CPU -->
<param name="localization_frequency" value="0.2" />
```

## 故障排查

### 问题1: 启动后无法找到地图文件
**症状**: 节点启动失败，报错"Cannot find map file"

**解决**:
```bash
# 检查地图文件是否存在
ls -lh /path/to/your/map.pcd

# 修改launch文件中的路径
nano src/FAST_LIO_LOCALIZATION/launch/wheel_odom_localization.launch
```

---

### 问题2: 设置初始位姿后定位失败
**症状**: 日志显示"Localization failed! Fitness too low"

**原因**: 初始位姿设置不准确

**解决**:
1. 在RViz中重新设置更准确的初始位姿
2. 确保机器人的朝向设置正确
3. 检查地图是否正确加载（查看RViz中的点云）

---

### 问题3: TF冲突警告
**症状**: 终端输出"TF_REPEATED_DATA ignoring data..."

**解决**:
```bash
# 检查是否有多个节点发布相同的TF
rosrun tf tf_monitor

# 确认底盘节点使用wheel_odom_frame
rostopic echo /wheel_odom | grep frame_id
# 应该显示: frame_id: "wheel_odom_frame"
```

---

### 问题4: 定位结果抖动严重
**症状**: `/localization`话题的位置剧烈跳动

**解决**:
编辑 `wheel_odom_localization.py`，增加平滑度：
```python
SMOOTH_ALPHA = 0.005  # 从0.01降低到0.005
```

---

### 问题5: 轮式里程计漂移严重
**症状**: 长时间运行后位置偏差很大

**解决**:
```xml
<!-- 增加配准次数 -->
<param name="max_registration_count" value="10" />

<!-- 或使用更高的定位频率 -->
<param name="localization_frequency" value="1.0" />
```

---

### 问题6: CPU占用过高
**症状**: 系统卡顿，CPU使用率>80%

**解决**:
```xml
<!-- 降低定位频率 -->
<param name="localization_frequency" value="0.3" />

<!-- 增大体素大小 -->
<param name="map_voxel_size" value="0.2" />
<param name="scan_voxel_size" value="0.1" />
```

## 性能监控

### 实时监控定位性能
```bash
# 监控话题频率
rostopic hz /localization
# 应该接近50Hz

rostopic hz /map_to_odom
# 应该接近0.5Hz

# 监控CPU使用
top -p $(pgrep -f wheel_odom_localization)
```

### 查看配准质量
```bash
# 查看fitness分数（越高越好）
rostopic echo /map_to_odom

# 正常情况fitness应该>0.8
```

## 日志说明

### 启动阶段
```
[ INFO] Wheel Odometry + Point Cloud Localization Node Started
[ INFO] Maximum registration count: 5
[ WARN] Waiting for global map......
[ INFO] Global map loaded! Points: 1234567
[ WARN] Waiting for initial pose....
```

### 配准阶段
```
[ INFO] Global localization by scan-to-map matching......
[ INFO] Registration time: 0.523s, fitness: 0.856
[ INFO] Localization succeeded! Fitness: 0.856
[ INFO] Registration count: 1/5
```

### 锁定阶段
```
[ WARN] ============================================================
[ WARN] LOCKED: Maximum registration count reached!
[ WARN] No more scan-to-map matching will be performed.
[ WARN] The system will now rely purely on wheel odometry.
[ WARN] ============================================================
```

## 高级用法

### 与导航包集成
```xml
<!-- move_base配置 -->
<node pkg="move_base" type="move_base" name="move_base">
    <!-- 使用全局定位结果 -->
    <remap from="odom" to="/localization"/>
    
    <!-- 坐标系设置 -->
    <param name="global_frame_id" value="map"/>
    <param name="base_frame_id" value="base_link"/>
</node>
```

### 录制定位数据
```bash
# 录制关键话题
rosbag record /localization /wheel_odom /map_to_odom /tf /tf_static -O localization_test.bag

# 回放测试
rosbag play localization_test.bag
```

### 性能分析
```bash
# 安装性能分析工具
sudo apt install ros-noetic-rqt-top

# 运行性能监控
rqt_top
```

## 下一步

- [ ] 调整参数以适应您的机器人和环境
- [ ] 录制测试数据验证定位精度
- [ ] 集成到导航系统中
- [ ] 根据需求修改配准次数限制

## 相关文档

- [详细README](./wheel_odom_localization_README.md)
- [坐标系修改说明](./coordinate_frame_modification.md)
- [FAST_LIO_LOCALIZATION官方文档](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)

## 技术支持

如遇问题，请检查：
1. 所有依赖包是否正确安装（open3d, ros-numpy等）
2. LiDAR驱动是否正常工作
3. 底盘通信是否正常（CAN总线）
4. 地图文件是否完整

祝使用愉快！🚀
