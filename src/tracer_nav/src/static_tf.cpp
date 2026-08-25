#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "static_tf");
  ros::NodeHandle nh;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  geometry_msgs::TransformStamped static_transformStamped;

  // 设置时间戳
  static_transformStamped.header.stamp = ros::Time::now();

  // 设置父坐标系
  static_transformStamped.header.frame_id = "body";

  // 设置子坐标系
  static_transformStamped.child_frame_id = "base_link";

  // 因为base_link在body前30cm，所以base_link在body坐标系的负x方向
  // 这里的 x, y, z 分别对应平移量
  static_transformStamped.transform.translation.x = -0.3;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.0;

  // 旋转量，这里假设没有旋转，使用单位四元数
  static_transformStamped.transform.rotation.x = 0.0;
  static_transformStamped.transform.rotation.y = 0.0;
  static_transformStamped.transform.rotation.z = 0.0;
  static_transformStamped.transform.rotation.w = 1.0;

  // 发布静态变换
  static_broadcaster.sendTransform(static_transformStamped);

  ROS_INFO("Successfully published static transform from 'body' to 'base_link'.");

  ros::spin();

  return 0;
}