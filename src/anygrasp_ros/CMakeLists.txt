cmake_minimum_required(VERSION 3.0.2)
project(anygrasp_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs visualization_msgs
)

install(PROGRAMS
  scripts/anygrasp_d405_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src/anygrasp_ros
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
