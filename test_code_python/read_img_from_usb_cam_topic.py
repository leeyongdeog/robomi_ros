#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ip128_usb_cam0/image_raw", Image, self.image_callback)
        self.save_interval = 60  # 1분마다 저장
        self.last_save_time = rospy.Time.now()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # 이미지 정보 파악 (예: 이미지 크기, 채널 수, 데이터 형식)
        image_info = {
            "height": cv_image.shape[0],
            "width": cv_image.shape[1],
            "channels": cv_image.shape[2],
            "dtype": cv_image.dtype
        }
        print("Image Info:", image_info)

        # 이미지 저장
        current_time = rospy.Time.now()
        if (current_time - self.last_save_time).to_sec() >= self.save_interval:
            filename = f"image_{current_time.to_sec()}.jpg"
            cv2.imwrite(filename, cv_image)
            print(f"Image saved: {filename}")
            self.last_save_time = current_time

if __name__ == "__main__":
    rospy.init_node("image_saver")
    image_saver = ImageSaver()
    rospy.spin()

