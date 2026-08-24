#include "mppi_local_planner/mppi_local_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace mppi_local_planner
{

namespace
{
constexpr float kPi = 3.14159265358979323846f;
}

MPPILocalPlanner::MPPILocalPlanner()
  : private_nh_("~")
{
}

void MPPILocalPlanner::initialize(std::string name,
                                   tf2_ros::Buffer* tf,
                                   costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) {
    ROS_WARN("MPPILocalPlanner has already been initialized");
    return;
  }

  private_nh_ = ros::NodeHandle("~" + name);
  nh_ = ros::NodeHandle();
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  planner_frame_ = costmap_ros_->getGlobalFrameID();

  private_nh_.param("odom_topic", odom_topic_, odom_topic_);
  private_nh_.param("scan_topic", scan_topic_, scan_topic_);
  private_nh_.param("scan_timeout", scan_timeout_, scan_timeout_);
  private_nh_.param("goal_tolerance", goal_tolerance_, goal_tolerance_);
  private_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, yaw_goal_tolerance_);
  private_nh_.param("prune_distance", prune_distance_, prune_distance_);

  configureController();

  odom_sub_ = nh_.subscribe(odom_topic_, 10, &MPPILocalPlanner::odomCallback, this);
  scan_sub_ = nh_.subscribe(scan_topic_, 2, &MPPILocalPlanner::scanCallback, this);
  debug_traj_pub_ = private_nh_.advertise<nav_msgs::Path>("debug_optimal_trajectory", 1);

  initialized_ = true;
  ROS_INFO("MPPILocalPlanner initialized: frame=%s odom=%s scan=%s",
           planner_frame_.c_str(), odom_topic_.c_str(), scan_topic_.c_str());
}

void MPPILocalPlanner::configureController()
{
  mppi::OptimizerSettings settings;
  int batch_size = 1500;
  int time_steps = 40;
  int iteration_count = 1;
  int thread_count = 4;
  private_nh_.param("batch_size", batch_size, batch_size);
  private_nh_.param("time_steps", time_steps, time_steps);
  private_nh_.param("iteration_count", iteration_count, iteration_count);
  private_nh_.param("thread_count", thread_count, thread_count);
  settings.batch_size = static_cast<unsigned int>(std::max(100, batch_size));
  settings.time_steps = static_cast<unsigned int>(std::max(5, time_steps));
  settings.iteration_count = static_cast<unsigned int>(std::max(1, iteration_count));
  settings.thread_count = static_cast<unsigned int>(std::max(1, thread_count));

  private_nh_.param("model_dt", settings.model_dt, 0.05f);
  private_nh_.param("temperature", settings.temperature, 0.20f);
  private_nh_.param("gamma", settings.gamma, 0.015f);
  private_nh_.param("vx_max", settings.base_constraints.vx_max, 0.35f);
  private_nh_.param("vx_min", settings.base_constraints.vx_min, 0.0f);
  private_nh_.param("vy_max", settings.base_constraints.vy_max, 0.0f);
  private_nh_.param("wz_max", settings.base_constraints.wz_max, 0.60f);
  private_nh_.param("ax_max", settings.base_constraints.ax_max, 0.20f);
  private_nh_.param("ay_max", settings.base_constraints.ay_max, 0.0f);
  private_nh_.param("az_max", settings.base_constraints.az_max, 0.35f);
  private_nh_.param("collision_cost_threshold",
                    settings.base_constraints.collision_cost_threshold, 5000.0f);
  settings.constraints = settings.base_constraints;

  private_nh_.param("vx_std", settings.sampling_std.vx, 0.12f);
  private_nh_.param("vy_std", settings.sampling_std.vy, 0.0f);
  private_nh_.param("wz_std", settings.sampling_std.wz, 0.20f);
  private_nh_.param("use_sg_filter", settings.use_sg_filter, false);
  private_nh_.param("shift_control_sequence", settings.shift_control_sequence, true);
  private_nh_.param("retry_attempt_limit", settings.retry_attempt_limit, 1);
  private_nh_.param("use_mean_normalization", settings.use_mean_normalization, false);
  private_nh_.param("adaptive_temperature", settings.adaptive_temperature, false);
  private_nh_.param("adaptive_temperature_min", settings.adaptive_temperature_min, 0.10f);
  private_nh_.param("adaptive_temperature_max", settings.adaptive_temperature_max, 1.00f);
  settings.prune_distance = prune_distance_;

  std::string motion_model;
  double ackermann_radius = 0.2;
  private_nh_.param("motion_model", motion_model, std::string("DiffDrive"));
  private_nh_.param("ackermann_min_turning_radius", ackermann_radius, 0.2);

  controller_ = std::make_unique<mppi::MPPIController>();
  controller_->initialize(settings, motion_model, static_cast<float>(ackermann_radius));

  double repulsion_weight = 0.5;
  double collision_cost = 10000.0;
  double collision_margin = 0.25;
  double inflation_radius = 0.8;
  double cost_scaling = 4.0;
  double near_goal_distance = 0.3;
  double robot_radius = 0.38;
  double grid_resolution = 0.05;
  int grid_width = 100;
  int grid_height = 100;
  bool consider_footprint = true;
  private_nh_.param("obstacle_repulsion_weight", repulsion_weight, repulsion_weight);
  private_nh_.param("obstacle_collision_cost", collision_cost, collision_cost);
  private_nh_.param("obstacle_collision_margin", collision_margin, collision_margin);
  private_nh_.param("obstacle_inflation_radius", inflation_radius, inflation_radius);
  private_nh_.param("obstacle_cost_scaling", cost_scaling, cost_scaling);
  private_nh_.param("obstacle_near_goal_distance", near_goal_distance, near_goal_distance);
  private_nh_.param("robot_radius", robot_radius, robot_radius);
  private_nh_.param("grid_resolution", grid_resolution, grid_resolution);
  private_nh_.param("grid_width", grid_width, grid_width);
  private_nh_.param("grid_height", grid_height, grid_height);
  private_nh_.param("consider_footprint", consider_footprint, consider_footprint);

  std::vector<double> footprint_values;
  private_nh_.param("footprint", footprint_values, footprint_values);
  std::vector<mppi::Point2D> footprint;
  for (size_t i = 0; i + 1 < footprint_values.size(); i += 2) {
    footprint.emplace_back(footprint_values[i], footprint_values[i + 1]);
  }
  if (controller_->getObstaclesCritic()) {
    controller_->getObstaclesCritic()->setParams(
        static_cast<float>(repulsion_weight), static_cast<float>(collision_cost),
        static_cast<float>(collision_margin), static_cast<float>(inflation_radius),
        static_cast<float>(cost_scaling), static_cast<float>(near_goal_distance),
        static_cast<float>(robot_radius), static_cast<float>(grid_resolution),
        grid_width, grid_height, consider_footprint, footprint);
  }

  double path_align_check_radius = 0.10;
  private_nh_.param("path_align_obstacle_check_radius", path_align_check_radius,
                    path_align_check_radius);
  controller_->setPathAlignObstacleCheckRadius(static_cast<float>(path_align_check_radius));

  if (controller_->getPathAlignCritic()) {
    double weight = 6.0;
    int offset = 16;
    double threshold = 0.40;
    int traj_step = 3;
    double max_occupancy_ratio = 0.50;
    bool use_orientations = false;
    private_nh_.param("path_align_weight", weight, weight);
    private_nh_.param("path_align_offset", offset, offset);
    private_nh_.param("path_align_threshold", threshold, threshold);
    private_nh_.param("path_align_traj_step", traj_step, traj_step);
    private_nh_.param("path_align_max_occupancy_ratio", max_occupancy_ratio,
                      max_occupancy_ratio);
    private_nh_.param("path_align_use_orientations", use_orientations, use_orientations);
    controller_->getPathAlignCritic()->setParams(
        weight, offset, threshold, traj_step, max_occupancy_ratio, use_orientations);
  }

  if (controller_->getPathAngleCritic()) {
    double weight = 2.0;
    int offset = 4;
    double threshold = 0.40;
    double angle_max = 0.5;
    int mode = 0;
    private_nh_.param("path_angle_weight", weight, weight);
    private_nh_.param("path_angle_offset", offset, offset);
    private_nh_.param("path_angle_threshold", threshold, threshold);
    private_nh_.param("path_angle_max", angle_max, angle_max);
    private_nh_.param("path_angle_mode", mode, mode);
    controller_->getPathAngleCritic()->setParams(weight, offset, threshold, angle_max, mode);
  }

  if (controller_->getPathFollowCritic()) {
    double weight = 4.0;
    int offset = 7;
    double threshold = 0.6;
    private_nh_.param("path_follow_weight", weight, weight);
    private_nh_.param("path_follow_offset", offset, offset);
    private_nh_.param("path_follow_threshold", threshold, threshold);
    controller_->getPathFollowCritic()->setParams(weight, offset, threshold);
  }

  if (controller_->getGoalCritic()) {
    double weight = 5.0;
    double threshold = 1.0;
    private_nh_.param("goal_weight", weight, weight);
    private_nh_.param("goal_threshold", threshold, threshold);
    controller_->getGoalCritic()->setParams(weight, threshold);
  }
  if (controller_->getGoalAngleCritic()) {
    double weight = 3.0;
    double threshold = 0.4;
    private_nh_.param("goal_angle_weight", weight, weight);
    private_nh_.param("goal_angle_threshold", threshold, threshold);
    controller_->getGoalAngleCritic()->setParams(weight, threshold);
  }
  if (controller_->getPreferForwardCritic()) {
    double weight = 5.0;
    double threshold = 0.5;
    private_nh_.param("prefer_forward_weight", weight, weight);
    private_nh_.param("prefer_forward_threshold", threshold, threshold);
    controller_->getPreferForwardCritic()->setParams(weight, threshold);
  }
  if (controller_->getConstraintCritic()) {
    double weight = 4.0;
    double c_vx_max = 0.35;
    double c_vx_min = 0.0;
    double c_vy_max = 0.0;
    double c_wz_max = 0.60;
    double min_turning_radius = 0.2;
    int motion_model_type = 0;
    private_nh_.param("constraint_weight", weight, weight);
    private_nh_.param("constraint_vx_max", c_vx_max, c_vx_max);
    private_nh_.param("constraint_vx_min", c_vx_min, c_vx_min);
    private_nh_.param("constraint_vy_max", c_vy_max, c_vy_max);
    private_nh_.param("constraint_wz_max", c_wz_max, c_wz_max);
    private_nh_.param("ackermann_min_turning_radius", min_turning_radius,
                      min_turning_radius);
    private_nh_.param("motion_model_type", motion_model_type, motion_model_type);
    controller_->getConstraintCritic()->setParams(
        weight, c_vx_max, c_vx_min, c_vy_max, c_wz_max,
        min_turning_radius, motion_model_type);
  }
  if (controller_->getVelocityDeadbandCritic()) {
    double weight = 35.0;
    double vx = 0.05;
    double vy = 0.05;
    double wz = 0.10;
    private_nh_.param("velocity_deadband_weight", weight, weight);
    private_nh_.param("velocity_deadband_vx", vx, vx);
    private_nh_.param("velocity_deadband_vy", vy, vy);
    private_nh_.param("velocity_deadband_wz", wz, wz);
    controller_->getVelocityDeadbandCritic()->setParams(weight, vx, vy, wz);
  }
  if (controller_->getTwirlingCritic()) {
    double weight = 10.0;
    double threshold = 0.5;
    private_nh_.param("twirling_weight", weight, weight);
    private_nh_.param("twirling_threshold", threshold, threshold);
    controller_->getTwirlingCritic()->setParams(weight, threshold);
  }

  ROS_INFO("MPPI controller configured: samples=%d horizon=%d dt=%.3f vx<=%.3f wz<=%.3f",
           batch_size, time_steps, settings.model_dt,
           settings.base_constraints.vx_max, settings.base_constraints.wz_max);
}

void MPPILocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  robot_speed_.vx = static_cast<float>(msg->twist.twist.linear.x);
  robot_speed_.vy = static_cast<float>(msg->twist.twist.linear.y);
  robot_speed_.wz = static_cast<float>(msg->twist.twist.angular.z);
}

void MPPILocalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (msg->header.frame_id.empty() || !tf_) {
    return;
  }

  geometry_msgs::TransformStamped scan_to_planner;
  try {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time(0) : msg->header.stamp;
    scan_to_planner = tf_->lookupTransform(
        planner_frame_, msg->header.frame_id, stamp, ros::Duration(0.05));
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(2.0, "MPPI cannot transform scan %s -> %s: %s",
                      msg->header.frame_id.c_str(), planner_frame_.c_str(), ex.what());
    return;
  }

  std::vector<mppi::Point2D> points;
  points.reserve(msg->ranges.size());
  float angle = msg->angle_min;
  for (const float range : msg->ranges) {
    if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max &&
        range <= 4.5f) {
      const tf2::Vector3 point_scan(range * std::cos(angle), range * std::sin(angle), 0.0);
      tf2::Transform transform;
      tf2::fromMsg(scan_to_planner.transform, transform);
      const tf2::Vector3 point_planner = transform * point_scan;
      points.emplace_back(static_cast<float>(point_planner.x()),
                          static_cast<float>(point_planner.y()));
    }
    angle += msg->angle_increment;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  scan_obstacles_ = std::move(points);
  scan_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
}

bool MPPILocalPlanner::getRobotPose(mppi::Pose2D& pose) const
{
  if (!costmap_ros_) {
    return false;
  }
  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    return false;
  }
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  yaw = tf2::getYaw(robot_pose.pose.orientation);
  pose.x = static_cast<float>(robot_pose.pose.position.x);
  pose.y = static_cast<float>(robot_pose.pose.position.y);
  pose.theta = static_cast<float>(yaw);
  return true;
}

bool MPPILocalPlanner::transformPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    std::vector<mppi::Pose2D>& transformed) const
{
  transformed.clear();
  transformed.reserve(plan.size());
  for (const auto& pose : plan) {
    geometry_msgs::PoseStamped transformed_pose;
    try {
      if (pose.header.frame_id.empty() || pose.header.frame_id == planner_frame_) {
        transformed_pose = pose;
      } else {
        tf_->transform(pose, transformed_pose, planner_frame_, ros::Duration(0.1));
      }
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "MPPI cannot transform global plan to %s: %s",
                        planner_frame_.c_str(), ex.what());
      return false;
    }
    transformed.emplace_back(
        static_cast<float>(transformed_pose.pose.position.x),
        static_cast<float>(transformed_pose.pose.position.y),
        static_cast<float>(tf2::getYaw(transformed_pose.pose.orientation)));
  }
  return !transformed.empty();
}

std::vector<mppi::Point2D> MPPILocalPlanner::currentObstacles() const
{
  std::vector<mppi::Point2D> obstacles;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const bool fresh_scan = !scan_obstacles_.empty() &&
                            (ros::Time::now() - scan_stamp_).toSec() <= scan_timeout_;
    if (fresh_scan) {
      return scan_obstacles_;
    }
  }

  // Before the first usable scan, fall back to lethal cells already present in
  // the local costmap. This prevents MPPI from planning through a known wall.
  if (!costmap_ros_ || !costmap_ros_->getCostmap()) {
    return obstacles;
  }
  const costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  const unsigned int stride = 2;
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); my += stride) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); mx += stride) {
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        double wx = 0.0;
        double wy = 0.0;
        costmap->mapToWorld(mx, my, wx, wy);
        obstacles.emplace_back(static_cast<float>(wx), static_cast<float>(wy));
      }
    }
  }
  return obstacles;
}

bool MPPILocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_ || plan.empty()) {
    return false;
  }

  std::vector<mppi::Pose2D> transformed;
  if (!transformPlan(plan, transformed)) {
    return false;
  }
  plan_ = std::move(transformed);
  goal_pose_ = plan_.back();
  controller_->setPath(plan_);
  controller_->reset();
  has_plan_ = true;
  ROS_INFO("MPPI received global plan with %zu poses", plan_.size());
  return true;
}

bool MPPILocalPlanner::goalReached(const mppi::Pose2D& pose) const
{
  const float distance = std::hypot(pose.x - goal_pose_.x, pose.y - goal_pose_.y);
  const float yaw_error = std::abs(shortestAngularDistance(pose.theta, goal_pose_.theta));
  return distance <= goal_tolerance_ && yaw_error <= yaw_goal_tolerance_;
}

bool MPPILocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  cmd_vel = geometry_msgs::Twist();
  if (!initialized_ || !has_plan_ || plan_.empty()) {
    return false;
  }

  mppi::Pose2D robot_pose;
  if (!getRobotPose(robot_pose)) {
    ROS_WARN_THROTTLE(2.0, "MPPI cannot obtain the current robot pose");
    return false;
  }
  if (goalReached(robot_pose)) {
    return true;
  }

  mppi::Twist2D robot_speed;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_speed = robot_speed_;
  }
  controller_->updateStaticObstacles(currentObstacles(), robot_pose);

  try {
    const mppi::Twist2D command = controller_->computeVelocityCommands(robot_pose, robot_speed);
    cmd_vel.linear.x = command.vx;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = command.wz;
  } catch (const std::exception& ex) {
    ROS_WARN_THROTTLE(1.0, "MPPI compute failed; stopping: %s", ex.what());
    controller_->reset();
    return true;
  }

  nav_msgs::Path debug_path;
  debug_path.header.stamp = ros::Time::now();
  debug_path.header.frame_id = planner_frame_;
  const auto trajectory = controller_->getOptimizedTrajectory();
  debug_path.poses.resize(trajectory.shape(0));
  for (size_t i = 0; i < trajectory.shape(0); ++i) {
    auto& pose = debug_path.poses[i];
    pose.header = debug_path.header;
    pose.pose.position.x = trajectory(i, 0);
    pose.pose.position.y = trajectory(i, 1);
    const float yaw = trajectory(i, 2);
    pose.pose.orientation.z = std::sin(yaw * 0.5f);
    pose.pose.orientation.w = std::cos(yaw * 0.5f);
  }
  debug_traj_pub_.publish(debug_path);
  return true;
}

bool MPPILocalPlanner::isGoalReached()
{
  if (!initialized_ || !has_plan_) {
    return false;
  }
  mppi::Pose2D robot_pose;
  return getRobotPose(robot_pose) && goalReached(robot_pose);
}

float MPPILocalPlanner::shortestAngularDistance(float from, float to)
{
  float difference = std::fmod(to - from + kPi, 2.0f * kPi);
  if (difference < 0.0f) {
    difference += 2.0f * kPi;
  }
  return difference - kPi;
}

}  // namespace mppi_local_planner

PLUGINLIB_EXPORT_CLASS(mppi_local_planner::MPPILocalPlanner, nav_core::BaseLocalPlanner)
