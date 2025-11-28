#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf2_ros

def publish_tf():
    rospy.init_node('tf_broadcaster_node')

    # 使用 tf2_ros 的静态广播器
    br = tf2_ros.StaticTransformBroadcaster()

    # 创建 TransformStamped 消息
    t = TransformStamped()
    t.header.frame_id = "ur_arm_wrist_3_link"       # 父坐标系
    t.child_frame_id = "target_point"        # 子坐标系






    xyz = [-0.01146465  ,0.01853642  ,0.41227329]
    q = [ 0.01316018 ,-0.22631526  ,0.97235897 ,-0.05591292]
    # 设置平移
    t.transform.translation.x = xyz[0]     # 1坐标系相对2坐标系的x
    t.transform.translation.y = xyz[1]
    t.transform.translation.z = xyz[2]

    # 设置旋转（四元数）
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]


    # 发布静态变换
    br.sendTransform(t)

    rospy.loginfo("Published static transform from frame_2 to frame_1")
    rospy.spin()

if __name__ == '__main__':
    publish_tf()
