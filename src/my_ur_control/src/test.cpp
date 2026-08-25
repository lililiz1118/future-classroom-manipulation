#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_ik_check");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 MoveGroup
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    std::string end_effector_link = arm.getEndEffectorLink();
    std::string reference_frame = "base_link";

    // 设置参考坐标系
    arm.setPoseReferenceFrame(reference_frame);

    // 设置目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.1876;
    target_pose.position.y = 0.0470;
    target_pose.position.z = 0.3289;
    target_pose.orientation.x = 0.70692;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.70729;

    // 创建 RobotState 对象
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(arm.getCurrentState()->getRobotModel()));
    const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup("manipulator");

    // 尝试求 IK
    bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, end_effector_link, 10, 0.1);

    if (found_ik)
    {
        ROS_INFO("IK solution found!");
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Joint values:");
        for (size_t i = 0; i < joint_values.size(); ++i)
            ROS_INFO("  joint[%lu] = %f", i, joint_values[i]);
    }
    else
    {
        ROS_WARN("No IK solution for the target pose!");
    }

    ros::shutdown();
    return 0;
}
