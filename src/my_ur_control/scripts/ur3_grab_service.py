#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from std_msgs.msg import Int32MultiArray

def array_publisher():
    rospy.init_node('array_publisher', anonymous=True)
    pub = rospy.Publisher('/int_array_topic', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        msg = Int32MultiArray()
        msg.data = [1, 2, 3, 4, 5]
        pub.publish(msg)
        rospy.loginfo(f"Published: {msg.data}")
        rate.sleep()

def sam_callback(self, msg):
    rospy.loginfo(f"Received message: {msg.data}")

    # 修改内容
    new_msg = String()
    new_msg.data = f"Received: {msg.data}"

    # 发布
    self.pub.publish(new_msg)
    rospy.loginfo(f"Published to /topic2: {new_msg.data}")

def graspnet_callback(self, msg):
    rospy.loginfo(f"Received message: {msg.data}")

    # 修改内容
    new_msg = String()
    new_msg.data = f"Received: {msg.data}"

    # 发布
    self.pub.publish(new_msg)
    rospy.loginfo(f"Published to /topic2: {new_msg.data}")

def ur_callback(self, msg):
    rospy.loginfo(f"Received message: {msg.data}")

    # 修改内容
    new_msg = String()
    new_msg.data = f"Received: {msg.data}"

    # 发布
    self.pub.publish(new_msg)
    rospy.loginfo(f"Published to /topic2: {new_msg.data}")

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('ur3_grab_service')

    # 创建发布者，发布到 /topic2
    state_pub = rospy.Publisher('/ur3_grab_state', Int32MultiArray, queue_size=10)
    input_pub = rospy.Publisher('/grap_input', String, queue_size=10)

    # 创建订阅者，订阅 /topic1
    rospy.Subscriber('/graspnet_pub', String, graspnet_callback)
    rospy.Subscriber('/sam_pub', String, sam_callback)
    rospy.Subscriber('/ur3_control_pub', String, ur_callback)

    rospy.loginfo("String relay node started, waiting for messages...")



    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 构造并发布 String 类型消息
        str_msg = String()
        str_msg.data = f"stuff"
        input_pub.publish(str_msg)

        # 构造并发布 Int32 类型消息
        msg = Int32MultiArray()
        msg.data = [1, 2, 3, 4, 5, 6]
        state_pub.publish(msg)



        # 控制循环频率
        rate.sleep()
