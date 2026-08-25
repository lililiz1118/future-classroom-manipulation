# 坐标系修改总结

## 修改原因
FAST-LIO已经发布了名为`odom`的坐标系，为避免冲突，将轮式里程计的参考坐标系改为`wheel_odom_frame`。

## 修改文件列表

### 1. 底盘驱动节点
**文件**: `/home/jt001/tracer_ws/src/tracer/tracer_ros/tracer_base/src/tracer_base_node.cpp`

**修改内容**:
```cpp
// 修改前
private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));

// 修改后
private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("wheel_odom_frame"));
```

**影响**: 
- 底盘节点发布的轮式里程计frame_id从`odom`改为`wheel_odom_frame`
- TF变换从 `odom→base_link` 改为 `wheel_odom_frame→base_link`

---

### 2. 定位节点
**文件**: `/home/jt001/tracer_ws/src/FAST_LIO_LOCALIZATION/scripts/wheel_odom_localization.py`

**修改内容**:

#### 修改点 1: map_to_odom消息
```python
# 修改前
map_to_odom.child_frame_id = 'odom'

# 修改后
map_to_odom.child_frame_id = 'wheel_odom_frame'
```

#### 修改点 2: TF广播
```python
# 修改前
br.sendTransform(trans, quat, local_wheel_odom.header.stamp, 'odom', 'map')

# 修改后
br.sendTransform(trans, quat, local_wheel_odom.header.stamp, 'wheel_odom_frame', 'map')
```

#### 修改点 3: odom_to_baselink消息
```python
# 修改前
odom_to_baselink.header.frame_id = 'odom'

# 修改后
odom_to_baselink.header.frame_id = 'wheel_odom_frame'
```

**影响**:
- 定位节点发布的TF从 `map→odom` 改为 `map→wheel_odom_frame`
- `/odom_to_baselink` 话题的frame_id改为`wheel_odom_frame`
- `/map_to_odom` 话题的child_frame_id改为`wheel_odom_frame`

---

### 3. Launch文件
**文件**: `/home/jt001/tracer_ws/src/FAST_LIO_LOCALIZATION/launch/wheel_odom_localization.launch`

**修改内容**:
```xml
<!-- 修改前 -->
<param name="odom_frame" value="odom" />

<!-- 修改后 -->
<param name="odom_frame" value="wheel_odom_frame" />
```

**影响**: 启动底盘节点时指定使用`wheel_odom_frame`作为里程计坐标系

---

## 坐标系关系对比

### 修改前 (有冲突)
```
FAST-LIO发布:
map → odom → body

底盘+定位发布:
map → odom → base_link

冲突: 两个系统都发布 map→odom 变换！
```

### 修改后 (无冲突)
```
FAST-LIO发布:
map → odom → body

底盘+定位发布:
map → wheel_odom_frame → base_link

静态TF:
body → base_link (偏移-0.255m)

无冲突: 两个独立的TF树！
```

## 完整TF树结构

```
map (全局地图坐标系)
 ├─ odom (FAST-LIO的里程计坐标系)
 │   └─ body (LiDAR中心，FAST-LIO发布)
 │       └─ base_link (底盘中心，静态TF)
 │
 └─ wheel_odom_frame (轮式里程计坐标系)
     └─ base_link (底盘中心，底盘节点发布)
```

## 话题变化

### /wheel_odom 话题
```yaml
# 修改前
header:
  frame_id: "odom"
child_frame_id: "base_link"

# 修改后
header:
  frame_id: "wheel_odom_frame"
child_frame_id: "base_link"
```

### /odom_to_baselink 话题
```yaml
# 修改前
header:
  frame_id: "odom"
child_frame_id: "base_link"

# 修改后
header:
  frame_id: "wheel_odom_frame"
child_frame_id: "base_link"
```

### /map_to_odom 话题
```yaml
# 修改前
header:
  frame_id: "map"
child_frame_id: "odom"

# 修改后
header:
  frame_id: "map"
child_frame_id: "wheel_odom_frame"
```

## 验证方法

### 1. 查看TF树
```bash
rosrun tf view_frames
evince frames.pdf
```
应该看到两个独立的分支：
- `map→odom→body→base_link`
- `map→wheel_odom_frame→base_link`

### 2. 查看TF变换
```bash
# FAST-LIO的坐标系
rosrun tf tf_echo map body

# 轮式里程计的坐标系
rosrun tf tf_echo map wheel_odom_frame
rosrun tf tf_echo wheel_odom_frame base_link
```

### 3. 查看话题内容
```bash
# 查看轮式里程计
rostopic echo /wheel_odom | grep frame_id

# 应该输出: frame_id: "wheel_odom_frame"
```

### 4. 检查TF监听
```bash
# 不应该有TF冲突警告
rosrun tf tf_monitor
```

## 注意事项

1. **编译**: 修改C++代码后需要重新编译
   ```bash
   cd ~/tracer_ws
   catkin_make
   ```

2. **Python脚本**: 修改Python脚本后无需编译，但要确保有可执行权限
   ```bash
   chmod +x scripts/wheel_odom_localization.py
   ```

3. **其他节点**: 如果有其他节点订阅`/wheel_odom`或使用轮式里程计TF，需要相应更新它们的frame_id

4. **导航包**: 如果使用move_base等导航包，可能需要配置使用`wheel_odom_frame`

## 兼容性说明

此修改确保：
- ✅ FAST-LIO继续正常工作（使用`odom`坐标系）
- ✅ 轮式里程计定位正常工作（使用`wheel_odom_frame`坐标系）
- ✅ 两个系统互不干扰
- ✅ 可以同时使用两种定位方式
- ✅ TF树结构清晰，无冲突

## 测试清单

- [ ] 编译tracer_base包成功
- [ ] 启动底盘节点，检查`/wheel_odom`话题的frame_id
- [ ] 启动定位launch文件，无TF冲突警告
- [ ] 在RViz中可视化TF树，确认两个独立分支
- [ ] 设置初始位姿，定位成功
- [ ] 查看`/localization`话题数据正常
- [ ] 运行`tf_monitor`，无警告信息

## 回滚方法

如需回滚到使用`odom`坐标系：

1. 恢复 `tracer_base_node.cpp`:
   ```cpp
   private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
   ```

2. 恢复 `wheel_odom_localization.py` 中所有 `'wheel_odom_frame'` 为 `'odom'`

3. 恢复 `wheel_odom_localization.launch`:
   ```xml
   <param name="odom_frame" value="odom" />
   ```

4. 重新编译并启动
