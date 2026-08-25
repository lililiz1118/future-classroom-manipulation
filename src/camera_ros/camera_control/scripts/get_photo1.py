import pyrealsense2 as rs
import numpy as np
import cv2
import os

# ctx = rs.context()
# for dev in ctx.devices:
#     print(dev.get_info(rs.camera_info.serial_number))

save_dir = os.path.join(os.path.expanduser("~"), "tracer_ws", "src", "camera_ros", "images")
os.makedirs(save_dir, exist_ok=True)

# 1. 创建 pipeline 和 config
pipeline = rs.pipeline()
config = rs.config()

# 2. 指定要使用的设备序列号
serial_number = "230322270831"  # 替换为你目标摄像头的序列号
config.enable_device(serial_number)

# 3. 配置流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 4. 启动相机
profile = pipeline.start(config)

# 深度后处理滤波器
spatial = rs.spatial_filter()          # 空间滤波，平滑噪点
temporal = rs.temporal_filter()        # 时间滤波，减少闪烁
hole_filling = rs.hole_filling_filter()  # 填补空洞

# 对深度图做对齐
align = rs.align(rs.stream.color)

# 自动曝光稳定以及时间滤波缓存需要预热多帧
warmup_frames = 10
for _ in range(warmup_frames):
    pipeline.wait_for_frames()

# 采样多帧后平均，进一步提升深度稳定性
capture_frames = 5
depth_sum = None
color_image = None
valid_frames = 0

for _ in range(capture_frames):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # 深度滤波顺序：空间 -> 时间 -> 填洞
    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)
    depth_frame = hole_filling.process(depth_frame)

    depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)

    if depth_sum is None:
        depth_sum = depth_image
    else:
        depth_sum += depth_image

    # 始终使用最新的彩色图像
    color_image = np.asanyarray(color_frame.get_data())
    valid_frames += 1

# 计算均值深度图
if depth_sum is None or color_image is None or valid_frames == 0:
    raise RuntimeError("未能获取到完整的彩色或深度图像，请检查相机连接。")

depth_image_avg = (depth_sum / valid_frames).astype(np.uint16)

# 为了便于检查，生成深度伪彩色图
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_avg, alpha=0.03), cv2.COLORMAP_JET)

# 保存到文件
cv2.imwrite(os.path.join(save_dir, "color.png"), color_image)
cv2.imwrite(os.path.join(save_dir, "depth.png"), depth_image_avg)
cv2.imwrite(os.path.join(save_dir, "depth_colormap.png"), depth_colormap)

# 输出关键采集信息
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"保存完成：color.png, depth.png, depth_colormap.png，路径：{save_dir}")
print(f"深度规模（米/单位）：{depth_scale}，平均帧数：{capture_frames}")

pipeline.stop()
