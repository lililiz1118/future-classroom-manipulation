roslaunch tracer_bringup tracer_robot_base.launch #底盘驱动文件，包含部分雷达可视化代码
roslaunch livox_ros_driver2 msg_MID360.launch  #雷达驱动包与另外的可视化代码，且与上一条冲突
roslaunch fast_lio_localization tracer_localization.launch #定位
roslaunch tracer_nav nav.launch #导航

fast_lio_localization发布从map到odom的变换
里程计(这里的里程计是谁？)发布从odom到body(这里是雷达坐标系)的变换；


TODO：
1、尝试发布从body到base_link的odometry消息，从而可使以base_link为目标点导航
2、调节teb参数
3、调节全局规划器的路径

