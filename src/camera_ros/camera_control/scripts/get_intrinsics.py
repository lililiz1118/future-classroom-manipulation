import pyrealsense2 as rs

# 创建pipeline并启动
pipeline = rs.pipeline()
config = rs.config()
config.enable_device("230322270831")    #d405
#config.enable_device("135122071659")      #d435i
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline_profile = pipeline.start(config)  # 返回 pipeline profile

# 等待几帧稳定
for i in range(5):
    frames = pipeline.wait_for_frames()

# 获取帧
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

# 获取彩色相机内参
color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
print("彩色相机内参:")
print("Width:", color_intrinsics.width)
print("Height:", color_intrinsics.height)
print("Fx:", color_intrinsics.fx)
print("Fy:", color_intrinsics.fy)
print("PPX:", color_intrinsics.ppx)
print("PPY:", color_intrinsics.ppy)
print("Distortion model:", color_intrinsics.model)
print("Coefficients:", color_intrinsics.coeffs)

# 构建 3x3 内参矩阵
K = [
    [color_intrinsics.fx, 0, color_intrinsics.ppx],
    [0, color_intrinsics.fy, color_intrinsics.ppy],
    [0, 0, 1]
]
print("Intrinsic matrix K:\n", K)

# 获取深度缩放因子
depth_sensor = pipeline_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth scale:", depth_scale, "meters per unit")

pipeline.stop()
