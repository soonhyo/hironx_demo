#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from dynamic_reconfigure.server import Server
from hironx_demo.cfg import ImageProcessingConfig

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor')

        self.exposure = 0
        self.brightness = 1.0
        self.contrast = 1.0

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/head_camera/rgb/image_processed', Image, queue_size=1)

        self.server = Server(ImageProcessingConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        self.exposure = config.exposure
        self.brightness = config.brightness
        self.contrast = config.contrast
        rospy.loginfo("Reconfigure Request: exposure=%d, brightness=%.2f, contrast=%.2f",
                      self.exposure, self.brightness, self.contrast)
        return config

    def apply_image_processing(self, cv_image):
        processed = cv2.convertScaleAbs(cv_image, alpha=self.contrast, beta=self.exposure)
        processed = cv2.convertScaleAbs(processed, alpha=self.brightness)
        return processed

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        processed = self.apply_image_processing(cv_image)

        # Publish the processed image
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding="bgr8")
            self.image_pub.publish(processed_msg)
        except Exception as e:
            rospy.logerr("Failed to convert and publish image: %s", e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    processor = ImageProcessor()
    processor.run()
