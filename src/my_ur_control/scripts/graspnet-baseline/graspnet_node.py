#!/home/jt001/.conda/envs/graspnet/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32
from std_msgs.msg import Int32MultiArray

from demo import demo
import os
    
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

task_start = 0
done = 0

def state_callback(self, msg):

    if msg.data[1] == 1:
        task_start = 1
    
    if msg.data[5] == 1:
        task_start = 0
        done = 0
    
    rospy.loginfo(f"Published to /topic2: {new_msg.data}")

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('ur3_grab_service')


    # 创建发布者，发布到 /topic2
    input_pub = rospy.Publisher('/grap_input', String, queue_size=10)

    # 创建订阅者，订阅 /topic1
    rospy.Subscriber('/ur3_grab_state', Int32MultiArray, state_callback)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not done and task_start:
            data_dir = os.path.join(ROOT_DIR, 'doc/d405_data/cola')
            demo(data_dir)
            done = 1

        # 控制循环频率
        rate.sleep()
    
    
