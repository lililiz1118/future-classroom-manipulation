cmake_minimum_required(VERSION 3.0.2)
project(yolo_world_ros)

find_package(catkin REQUIRED COMPONENTS
  anygrasp_ros
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS anygrasp_ros geometry_msgs rospy sensor_msgs std_msgs tf tf2_ros
)

install(PROGRAMS
  scripts/yolo_target_cloud_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_target_cloud.py)
  catkin_add_nosetests(test/test_node_runtime.py)
  catkin_add_nosetests(test/test_launch_config.py)
  catkin_add_nosetests(test/test_package_manifest.py)
endif()
