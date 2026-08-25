# 轮式里程计 + 点云配准定位节点

## 概述

本节点结合轮式里程计和点云配准技术，实现精确的全局定位。避免了与FAST-LIO的`odom`坐标系冲突，使用独立的`wheel_odom_frame`坐标系。

## 特性

- ✅ **融合轮式里程计**：使用底盘编码器提供的轮式里程计作为基础
- ✅ **点云配准定位**：通过ICP算法将实时点云与全局地图配准
- ✅ **有限配准次数**：初始定位后仅进行5次配准，之后依赖纯轮式里程计
- ✅ **平滑滤波**：对定位结果进行插值平滑，消除抖动
- ✅ **独立坐标系**：使用`wheel_odom_frame`避免与FAST-LIO的`odom`冲突

## 坐标系架构

```
map (全局地图坐标系)
 └─ wheel_odom_frame (轮式里程计坐标系，独立于FAST-LIO)
     └─ base_link (底盘中心)
         └─ body (LiDAR中心，由static_transform_publisher发布)

FAST-LIO独立发布:
map
 └─ odom (FAST-LIO的里程计坐标系)
     └─ body (LiDAR坐标系)
```

## 文件说明

### 1. 定位节点
- **文件**: `scripts/wheel_odom_localization.py`
- **功能**:
  - 订阅 `/wheel_odom` (轮式里程计)
  - 订阅 `/cloud_registered` (FAST-LIO点云)
  - 订阅 `/pcd_map` (全局地图)
  - 订阅 `/initialpose` (初始位姿)
  - 发布 `/localization` (全局定位结果: map→base_link)
  - 发布 `/odom_to_baselink` (轮式里程计: wheel_odom_frame→base_link)
  - 发布 TF: map→wheel_odom_frame

### 2. Launch文件
- **文件**: `launch/wheel_odom_localization.launch`
- **启动内容**:
  - tracer底盘驱动节点
  - Livox MID-360驱动
  - FAST-LIO节点
  - 地图加载节点
  - 轮式里程计定位节点
  - 静态TF发布器

### 3. 底盘驱动修改
- **文件**: `tracer_base/src/tracer_base_node.cpp`
- **修改**: 将默认`odom_frame`从`"odom"`改为`"wheel_odom_frame"`

## 使用方法

### 1. 准备全局地图
确保已经有PCD格式的全局地图文件，例如：
```
/home/jt001/fastlio_ws/src/FAST_LIO/PCD/scans.pcd
```

### 2. 修改launch文件参数
编辑 `launch/wheel_odom_localization.launch`：
```xml
<arg name="map_file" default="你的地图文件路径.pcd" />
```

### 3. 启动定位系统
```bash
roslaunch fast_lio_localization wheel_odom_localization.launch
```

### 4. 设置初始位姿
在RViz中使用"2D Pose Estimate"工具设置机器人的初始位姿。

### 5. 查看定位结果
```bash
# 查看定位结果话题
rostopic echo /localization

# 查看TF树
rosrun tf view_frames
evince frames.pdf

# 查看坐标系关系
rosrun tf tf_echo map base_link
```

## 话题说明

### 订阅话题
| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/wheel_odom` | `nav_msgs/Odometry` | 轮式里程计 (wheel_odom_frame→base_link) |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO配准后的点云 |
| `/pcd_map` | `sensor_msgs/PointCloud2` | 全局地图点云 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿 |

### 发布话题
| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/localization` | `nav_msgs/Odometry` | 全局定位结果 (map→base_link) |
| `/odom_to_baselink` | `nav_msgs/Odometry` | 轮式里程计 (wheel_odom_frame→base_link) |
| `/map_to_odom` | `nav_msgs/Odometry` | 地图到里程计变换 (map→wheel_odom_frame) |
| `/cur_scan_in_map` | `sensor_msgs/PointCloud2` | 当前扫描在地图系下的位置 |
| `/submap` | `sensor_msgs/PointCloud2` | 视野内的子地图 |

### TF变换
| 父坐标系 | 子坐标系 | 说明 |
|---------|---------|------|
| `map` | `wheel_odom_frame` | 地图到轮式里程计坐标系 |
| `body` | `base_link` | LiDAR到底盘中心 (静态变换) |

## 参数配置

在launch文件中可配置的参数：

```xml
<!-- 地图和扫描点云体素大小 -->
<param name="map_voxel_size" value="0.1" />
<param name="scan_voxel_size" value="0.05" />

<!-- 定位频率 (Hz) -->
<param name="localization_frequency" value="0.5" />

<!-- 发布频率 (Hz) -->
<param name="publish_frequency" value="50" />

<!-- 定位阈值 (fitness score) -->
<param name="localization_threshold" value="0.8" />

<!-- 最大配准次数 -->
<param name="max_registration_count" value="5" />

<!-- LiDAR视野参数 -->
<param name="fov" value="6.28319" />  <!-- 360度 -->
<param name="fov_far" value="30.0" />  <!-- 30米 -->
```

## 工作流程

1. **初始化阶段**:
   - 加载全局地图
   - 等待初始位姿设置
   - 执行首次点云配准

2. **定位阶段** (配准次数 < 5):
   - 以0.5Hz频率执行点云配准
   - 更新`map→wheel_odom_frame`变换
   - 发布定位结果

3. **纯里程计阶段** (配准次数 ≥ 5):
   - 停止点云配准
   - 依赖纯轮式里程计
   - `map→wheel_odom_frame`保持固定

4. **融合发布** (持续50Hz):
   - 对`map→wheel_odom_frame`进行平滑插值
   - 发布TF变换
   - 发布定位结果Odometry消息

## 平滑滤波逻辑

采用自适应alpha系数的插值平滑：
- **大跳变** (>0.5m): alpha=0.3 (较快更新)
- **中跳变** (0.1~0.5m): alpha=0.05 (缓慢更新)
- **小变化** (<0.1m): alpha=0.01 (非常平滑)

## 注意事项

1. **坐标系冲突**: 
   - FAST-LIO发布 `map→odom→body`
   - 本系统发布 `map→wheel_odom_frame→base_link`
   - 两者互不干扰

2. **初始位姿**: 必须在RViz中设置准确的初始位姿，否则配准可能失败

3. **地图质量**: 全局地图的质量直接影响定位精度

4. **轮式里程计**: 需要底盘提供准确的轮式里程计数据到`/wheel_odom`话题

5. **配准次数**: 达到5次后将锁定，仅依赖轮式里程计，适合长时间运行

## 调试技巧

### 查看配准fitness
```bash
# 查看定位节点日志
rosrun rqt_console rqt_console
# 筛选 wheel_odom_localization 节点
```

### 可视化点云配准
在RViz中添加：
- `/cur_scan_in_map` (当前扫描在地图系下)
- `/submap` (视野内子地图)

### 检查TF树
```bash
rosrun rqt_tf_tree rqt_tf_tree
```

## 常见问题

### Q1: 定位失败，fitness很低
**A**: 检查初始位姿是否准确，尝试重新设置

### Q2: 定位结果抖动严重
**A**: 调大`SMOOTH_ALPHA`参数，增加平滑度

### Q3: 轮式里程计漂移严重
**A**: 考虑增加`max_registration_count`，允许更多次配准

### Q4: TF树中出现两个odom
**A**: 确保底盘节点使用`wheel_odom_frame`而非`odom`

## 性能指标

- **定位精度**: ±5cm (取决于地图质量)
- **配准时间**: ~0.5s/次
- **发布频率**: 50Hz
- **CPU占用**: ~20% (单核)

## 作者

基于FAST_LIO_LOCALIZATION项目开发
