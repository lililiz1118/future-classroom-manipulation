#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np


class MoveItIkControl:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'ur_arm_base_link'  #ur_arm_base_link -->  ur_arm_wrist_3_link
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.1)
        arm.set_max_velocity_scaling_factor(0.1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('one')
        print("send original state")
        arm.go()
        rospy.sleep(1)

               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于ur_arm_base_link坐标系

        # 设置目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = -0.15566759
        target_pose.pose.position.y = 0.35226438
        target_pose.pose.position.z = 0.38237145

        # 四元数归一化

        q = np.array([ 0.73671231 ,-0.05448221 , 0.04970928 ,-0.67217234])
        q = q / np.linalg.norm(q)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        print("send 2 goal")
        # 规划运动路径
        success, traj, _, _ = arm.plan()
      
        # 根据规划结果决定是否执行
        if success:
            # 按照规划的运动路径控制机械臂运动
            # arm.execute(traj)
            rospy.sleep(1)
        else:
            rospy.logerr("第二次运动规划失败！")

        # 控制机械臂回到初始化位置
        arm.set_named_target('one')
        print("send 3 goal")
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('moveit_ik_demo')

    MoveItIkControl()
    # arm = moveit_commander.MoveGroupCommander("arm")
    # print("控制的关节列表:", arm.get_active_joints())


# def MoveItIkControl(target_frame,  target_xyz, target_quaternion, execution = 1):
#     # 初始化move_group的API
#     moveit_commander.roscpp_initialize(sys.argv)
            
#     arm = moveit_commander.MoveGroupCommander('arm')

#     end_effector_link = arm.get_end_effector_link()
#     print(end_effector_link)
                    
#     reference_frame = target_frame  #ur_arm_base_link ur_arm_wrist_3_link
#     arm.set_pose_reference_frame(reference_frame)
            
#     # 当运动规划失败后，允许重新规划
#     arm.allow_replanning(True)
    
#     # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
#     arm.set_goal_position_tolerance(0.01)
#     arm.set_goal_orientation_tolerance(0.05)
    
#     # 设置允许的最大速度和加速度
#     arm.set_max_acceleration_scaling_factor(0.1)
#     arm.set_max_velocity_scaling_factor(0.1)
            
#     target_pose = PoseStamped()
#     target_pose.header.frame_id = reference_frame
#     target_pose.header.stamp = rospy.Time.now()

#     target_pose.pose.position.x = target_xyz[0]
#     target_pose.pose.position.y = target_xyz[1]
#     target_pose.pose.position.z = target_xyz[2]

#     # 四元数归一化
#     q = target_quaternion
#     q = q / np.linalg.norm(q)
#     target_pose.pose.orientation.x = q[0]
#     target_pose.pose.orientation.y = q[1]
#     target_pose.pose.orientation.z = q[2]
#     target_pose.pose.orientation.w = q[3]

#     # 设置机器臂当前的状态作为运动初始状态
#     arm.set_start_state_to_current_state()
    
#     # 设置机械臂终端运动的目标位姿
#     arm.set_pose_target(target_pose, end_effector_link)
#     print("send taregt goal")
    
#     success, traj, _, _ = arm.plan()

#     print(len(traj.joint_trajectory.points))

#     # while not success:
#     #     success, traj, _, _ = arm.plan()
#     #     print(len(traj.joint_trajectory.points))
#     #     rospy.logerr("第二次运动规划失败！")

#     # is_success = 0
#     # round = 0
#     # traj_list = []
#     # while not is_success:
#     #     rospy.loginfo("planning")
#     #     success, traj, _, _ = arm.plan()
#     #     print(len(traj.joint_trajectory.points))

#     #     if len(traj.joint_trajectory.points) <= 20 and len(traj.joint_trajectory.points) != 0 and success:
#     #         is_success = 1
#     #     else:
#     #         rospy.loginfo("planning failed, try again")
#     #         rospy.sleep(1)

#     #     round += 1
#     #     traj_list.append(traj)

#     #     if round >= 5:
#     #         min = 200
#     #         min_index = 0
#     #         for i in range(5):
#     #             if len(traj[i].joint_trajectory.points) < min:
#     #                 min = traj[i].joint_trajectory.points
#     #                 min_index = i

            
#     #         traj = traj_list[min_index]
#     #         break


#     # 按照规划的运动路径控制机械臂运动
#     print("success")

#     if execution:
#         arm.execute(traj)

#     rospy.sleep(1)
#     print("Target pose (TCP frame):", target_pose)
#     print(target_pose)
#     # print(target_xyz)
#     current_pose = arm.get_current_pose(end_effector_link).pose
#     print("Current pose:", current_pose)
#     print("Planning frame:", arm.get_planning_frame())