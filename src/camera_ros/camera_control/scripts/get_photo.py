import pyrealsense2 as rs
import numpy as np
import cv2
import os

# ctx = rs.context()
# for dev in ctx.devices:
#     print(dev.get_info(rs.camera_info.serial_number))

save_dir = "/home/jt001/tracer_ws/src/camera_ros/images"

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
pipeline.start(config)

# 等待几帧稳定
for i in range(5):
    frames = pipeline.wait_for_frames()

# 获取图像
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

color_image = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())

# 保存到文件
cv2.imwrite(os.path.join(save_dir, "color.png"), color_image)
cv2.imwrite(os.path.join(save_dir, "depth.png"), depth_image)

print("保存完成：color.png, depth.png")

pipeline.stop()
