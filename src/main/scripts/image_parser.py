#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        rospy.init_node('image_parser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False
        self.img_bgr = None

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                cv2.destroyAllWindows()
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                cv2.imshow("Image window", self.img_bgr)
                cv2.waitKey(1)
                print(f"Caemra sensor was connected !")

            rate.sleep()


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    



if __name__ == '__main__':
    try:
        IMGParser = IMGParser()
    except rospy.ROSInterruptException:
        pass
