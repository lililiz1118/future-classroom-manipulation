#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse
import cv2
import os
import threading

SAVE_DIR = "/home/jt001/realsense_images/d405"
os.makedirs(SAVE_DIR, exist_ok=True)


class RealsenseImageSaverService:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_msg = None
        self.depth_msg = None
        self.lock = threading.Lock()

        # 建立服务
        self.service = rospy.Service("/d405_save_realsense_images", Trigger, self.handle_save)

        rospy.loginfo("✅ RealsenseImageSaverService 已启动，可通过 call /d405_save_realsense_images 保存图像。")

    def color_callback(self, msg):
        with self.lock:
            self.color_msg = msg

    def depth_callback(self, msg):
        with self.lock:
            self.depth_msg = msg
            
    def handle_save(self, req):
        try:
            # 临时订阅
            color_msg = rospy.wait_for_message("/d405/color/image_raw", Image, timeout=2.0)
            depth_msg = rospy.wait_for_message("/d405/depth/image_rect_raw", Image, timeout=2.0)

            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

            color_path = os.path.join(SAVE_DIR, "color.png")
            depth_path = os.path.join(SAVE_DIR, "depth.png")

            cv2.imwrite(color_path, color_image)
            cv2.imwrite(depth_path, depth_image)

            msg = f"已保存: {os.path.basename(color_path)}, {os.path.basename(depth_path)}"
            rospy.loginfo(msg)
            return TriggerResponse(success=True, message=msg)

        except Exception as e:
            rospy.logerr(f"保存图像出错: {e}")
            return TriggerResponse(success=False, message=str(e))


if __name__ == "__main__":
    rospy.init_node("d405_realsense_image_saver_service")
    saver = RealsenseImageSaverService()
    rospy.spin()
