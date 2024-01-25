#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import CamShower

class cam_sub():
    def __init__(self) -> None:
        rospy.init_node("sim_cam_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self._img_cb)
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
    
    def _img_cb(self, msg):
        self.image_msg = msg
        cv_img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
        cam = CamShower.Camtest()
        cam.ros = 1
        cam.wait_time = 1
        cam.imghsv(cv_img)
        



def main():
    try:
        s = cam_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()