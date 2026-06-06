#!/usr/bin/env python3
import rospy
import tf
import copy
from nav_msgs.msg import Odometry
import math

class OdomVelocityEstimator:
    def __init__(self):
        rospy.init_node('odom_velocity_fixer')

        # 参数设置
        self.input_topic = rospy.get_param('~input_topic', '/localization')
        self.output_topic = rospy.get_param('~output_topic', '/localization_speed')
        self.alpha = rospy.get_param('~alpha', 0.25)  # 越小越平滑，但延迟越大
        self.min_dt = rospy.get_param('~min_dt', 0.005)
        self.max_dt = rospy.get_param('~max_dt', 0.30)  # dt过大通常意味着断流/跳变，直接重置避免速度尖峰

        self.last_odom = None
        self.v_filt = 0.0
        self.w_filt = 0.0

        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=20)
        # 发布带速度的新里程计
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)

        rospy.loginfo("fastlio_speed: input_topic=%s output_topic=%s alpha=%.3f min_dt=%.3f max_dt=%.3f",
                      self.input_topic, self.output_topic, self.alpha, self.min_dt, self.max_dt)

    def odom_callback(self, msg):
        if self.last_odom is None:
            self.last_odom = msg
            return

        # 1. 计算时间差
        dt = (msg.header.stamp - self.last_odom.header.stamp).to_sec()
        if dt <= self.min_dt:
            return
        if dt > self.max_dt:
            # 输入断流后第一次恢复时，跳过该帧，避免速度被一次大位移放大。
            self.last_odom = msg
            return

        # 2. 计算位移 (世界坐标系)
        dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
        
        # 3. 计算角度差 (Yaw)
        q_old = [self.last_odom.pose.pose.orientation.x, self.last_odom.pose.pose.orientation.y,
                 self.last_odom.pose.pose.orientation.z, self.last_odom.pose.pose.orientation.w]
        q_new = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        
        yaw_old = tf.transformations.euler_from_quaternion(q_old)[2]
        yaw_new = tf.transformations.euler_from_quaternion(q_new)[2]
        
        dyaw = yaw_new - yaw_old
        # 角度突变处理 (-PI 到 PI)
        if dyaw > math.pi: dyaw -= 2*math.pi
        if dyaw < -math.pi: dyaw += 2*math.pi

        # 4. 基础速度计算
        # 计算在局部坐标系 (last_odom) 下的 X 方向和 Y 方向标量速度
        # 利用旋转矩阵将世界系的位移 (dx, dy) 投影到上一次的朝向 yaw_old 上
        vx_raw = (dx * math.cos(yaw_old) + dy * math.sin(yaw_old)) / dt
        vy_raw = (-dx * math.sin(yaw_old) + dy * math.cos(yaw_old)) / dt
        
        # 对于底盘来说，vx_raw才是我们真正想要关注的前进/后退速度
        # 抛弃绝对距离的模和 atan2 计算方法，这样能完全消除微小滑移带来的纯旋转时的信号翻转噪声
        v_raw = vx_raw
        
        w_raw = dyaw / dt

        # 5. 低通滤波 (消除差分带来的高频噪声)
        self.v_filt = self.alpha * v_raw + (1 - self.alpha) * self.v_filt
        self.w_filt = self.alpha * w_raw + (1 - self.alpha) * self.w_filt

        # 6. 填充并发布新消息
        new_msg = copy.deepcopy(msg)
        new_msg.twist.twist.linear.x = self.v_filt
        new_msg.twist.twist.angular.z = self.w_filt
        # 给一个小的速度协方差，防止规划器报错
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 6 + i] = 0.1
        new_msg.twist.covariance = cov
        
        self.pub.publish(new_msg)
        self.last_odom = msg

if __name__ == '__main__':
    try:
        OdomVelocityEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass