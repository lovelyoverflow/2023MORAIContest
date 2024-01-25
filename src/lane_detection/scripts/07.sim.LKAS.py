#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import CamShower #Local file
from sim_kbd import Sim_kbd
from math import isnan

# does this even work


class Cam_sub():
    def __init__(self) -> None:
        rospy.init_node("sim_lkas_node")
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
        self.cam = CamShower.Camtest()
        self.cam.ros = 1
        self.cam.wait_time = 1
        turn = 0.2
        self.kbd = Sim_kbd(motor_spdw = 1500, servo_l = 0.5-turn, servo_r = 0.5+turn)
        
        self.nwindows = 9
        self.margin = 60
        self.minpix = 5
    
    def subscribe(self):
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self._img_cb, queue_size = 1)
    
    def warp_process_image(self, img): # does this even work
        
        
        nwindows = self.nwindows
        margin = self.margin
        minpix = self.minpix

        lane = img

        histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)      
        midpoint = np.int(histogram.shape[0]/2)
        leftx_current = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint

        window_height = np.int(lane.shape[0]/nwindows)
        nz = lane.nonzero() # nz[0] is vertical(y), nz[1] is horizontal(x)

        left_lane_inds = []
        right_lane_inds = []
        
        lx, ly, rx, ry = [], [], [], []

        out_img = np.dstack((lane, lane, lane))*255

        for window in range(nwindows):

            win_yl = lane.shape[0] - (window+1)*window_height
            win_yh = lane.shape[0] - window*window_height

            win_xll = leftx_current - margin
            win_xlh = leftx_current + margin
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

            good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nz[1][good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nz[1][good_right_inds]))

            lx.append(leftx_current)
            ly.append((win_yl + win_yh)/2)

            rx.append(rightx_current)
            ry.append((win_yl + win_yh)/2)

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
        cv2.imshow("viewer", out_img)
        
        #return left_fit, right_fit
        return (lx, ly), (rx, ry)


    def _img_cb(self, msg):
        self.image_msg = msg
        cv_img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
        
        y, x, _ = cv_img.shape
        img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)

        # seperate lanes
        lower = np.array([15, 100, 140])
        upper = np.array([30, 200, 255])
        yellow_range = cv2.inRange(img_hsv, lower, upper)

        lowerwhite = np.array([0, 0, 140])
        upperwhite = np.array([50, 70, 255])
        white_range = cv2.inRange(img_hsv, lowerwhite, upperwhite)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_image = cv2.bitwise_and(cv_img, cv_img, mask = combined_range)

        # Warp img to ROI(warped img)
        topx = 269
        bottomx = -25
        topy = 271
        bottomy = y
        src_points = np.float32([[bottomx, bottomy], [topx, topy], [x - topx, topy], [x-bottomx, bottomy]])
        # print(src_points)
        topx = x//6
        bottomx = x//6
        topy = 0
        bottomy = x
        dst_points = np.float32([[bottomx, bottomy], [topx, topy], [x - topx, topy], [x-bottomx, bottomy]])
        # print(dst_points)
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        # print(matrix)
        warped_img = cv2.warpPerspective(combined_range, matrix, [x, x])

        # WTF is this
        posl, posr = self.warp_process_image(warped_img)
        
        posl = int(posl[0][2])
        posr = int(posr[0][2])
        
        cv2.line(warped_img, [posl, 0], [posl, x-1], 255, 5)
        cv2.line(warped_img, [posr, 0], [posr, x-1], 255, 5)

        cv2.namedWindow("warped", cv2.WINDOW_NORMAL)
        cv2.imshow("warped", warped_img)

        self.cam._cv2_wait(())

        # start of ctrl stuff
        midrange = 300
        midstart = x//2-midrange
        midend = x//2+midrange
        mid = x//2
        pos = (posl + posr) // 2
        if not isnan(posr):
            ctrl = (((pos - midstart) * (1 - 0)) / (midend - midstart)) + 0
        else: ctrl = 0.5 # ctrl = self.cam.keycode
        self.kbd.steer(ctrl)

        

        
def main():
    try:
        s = Cam_sub()
        s.subscribe()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()