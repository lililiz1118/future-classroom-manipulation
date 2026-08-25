#ifndef MPPI_LOCAL_PLANNER_MPPI_LOCAL_PLANNER_H_
#define MPPI_LOCAL_PLANNER_MPPI_LOCAL_PLANNER_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>

#include "controller.hpp"

namespace mppi_local_planner
{

class MPPILocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  MPPILocalPlanner();
  ~MPPILocalPlanner() override = default;

  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  void configureController();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  bool getRobotPose(mppi::Pose2D& pose) const;
  bool transformPlan(const std::vector<geometry_msgs::PoseStamped>& plan,
                     std::vector<mppi::Pose2D>& transformed) const;
  std::vector<mppi::Point2D> currentObstacles() const;
  bool goalReached(const mppi::Pose2D& pose) const;
  static float shortestAngularDistance(float from, float to);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

  std::unique_ptr<mppi::MPPIController> controller_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher debug_traj_pub_;

  mutable std::mutex data_mutex_;
  mppi::Twist2D robot_speed_;
  std::vector<mppi::Point2D> scan_obstacles_;
  ros::Time scan_stamp_;

  std::string planner_frame_;
  std::string odom_topic_ = "/localization_speed";
  std::string scan_topic_ = "/scan";
  double scan_timeout_ = 0.5;
  float goal_tolerance_ = 0.15f;
  float yaw_goal_tolerance_ = 0.20f;
  float prune_distance_ = 2.0f;

  std::vector<mppi::Pose2D> plan_;
  mppi::Pose2D goal_pose_;
  bool initialized_ = false;
  bool has_plan_ = false;
};

}  // namespace mppi_local_planner

#endif  // MPPI_LOCAL_PLANNER_MPPI_LOCAL_PLANNER_H_
