cmake_minimum_required(VERSION 3.0.2)
project(mppi_local_planner)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(MPPI_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../SA-MPPI/src/mppi_laser_example)
set(RC_ESDF_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../SA-MPPI/src/RC-ESDF)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp sensor_msgs tf tf2 tf2_geometry_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MPPI_ROOT}/include
  ${MPPI_ROOT}/xtensor/include
  ${MPPI_ROOT}/xsimd/include
  ${MPPI_ROOT}/xtl/include
  ${RC_ESDF_ROOT}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mppi_local_planner.cpp
  ${RC_ESDF_ROOT}/src/rc_esdf.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES mppi_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
