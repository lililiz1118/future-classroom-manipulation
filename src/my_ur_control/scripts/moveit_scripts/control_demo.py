import moveit_commander

# 初始化
moveit_commander.roscpp_initialize([])

# 创建 MoveGroupCommander 对象
arm = moveit_commander.MoveGroupCommander('arm')

# 获取末端执行器名称（可选）
end_effector_link = arm.get_end_effector_link()
print("End-effector link:", end_effector_link)

# 获取当前末端位姿
current_pose = arm.get_current_pose(end_effector_link)
# print("Current Pose:\n", current_pose)

# 如果只想要 position 或 orientation
position = current_pose.pose.position
orientation = current_pose.pose.orientation
print("Position:", position.x, position.y, position.z)
print("Orientation (quaternion):", orientation.x, orientation.y, orientation.z, orientation.w)
