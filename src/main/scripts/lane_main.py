#!/usr/bin/env python3
#-*- coding:utf-8 -*-

###판단 부분부터 포매팅 신경안쓰고 작업, img warp은 추가사항 있음

import numpy as np
import cv2
import time
import math
import rospy
from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import GetTrafficLightStatus
from cv_bridge import CvBridge
# from pidcal import Pidcal

from std_msgs.msg import Float64

from math import isnan
import statistics

from move_base_msgs.msg import MoveBaseGoal

class LaneFollower:
    def __init__(self) -> None:

        #Image Data
        self.cv_img = None

        self.img = None
        self.img_hsv = None
        self.y, self.x = None, None
        self.h, self.s, self.v = None, None, None
        self.img_backup = None

        #img_transform
        self.yellow_range = None
        self.white_range = None
        self.combined_range = None

        #img_warp
        self.warp_img_size = None
        self.warp_img_zoomx = None
        self.warped_img = None

        #yellow
        self.yellow_warped = None

        #sliding_window
        self.out_img = None
        self.nwindows = 12
        self.margin = 60
        self.minpix = 5
        self.threshold = 100
        self.l_lane, self.r_lane = None, None

        #판단 공통
        self.pos = None
        self.trustr = True
        self.stopline_toggle = 4
        self.stopline_count = 0
        self.seq5_toggle = False
        self.tmp_time = 0
        self.temp_elapsed_time = 0
        
        self.yellowline_toggle = 4
        self.yellowline_count = 0
        self.count = 0
        self.sequence = -1
        self.cut_img_top = False
        self.yellow_based_slidingwindow = False
        self.steering_msg = Float64()
        self.speed_msg = Float64()

        #control_pub
        self.midrange = 300
        self.speed = None
        self.directControl = None

        self.lane_msg = Float64()
        # obstacle detector
        self.obstacle_state = ""
        self.current_lane = "RIGHT"
        self.is_safe = True
        self.dynamic_flag = False
        self.dynamic_mission = False
        self.DynamicDrive = False

        # static obstacle mission
        self.changed_lane_toggle = False
        self.static_flag = False
        self.static_mission = False
        self.turn_left_flag = 0
        self.turn_right_flag = 0
        
        self.static_sequence = 0

        # lidar warning string
        self.lidar_warning = ""

        self.y_list = []
        self.y_list_sort = []

        # self.pidcal = Pidcal()
        self.obstacle_t1 = 0
        #misc
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()
        self.left_traffic = False
    
    
    ##인지
    def img_init(self, img):# -> img,img_hsv,x,y,h,s,v
        self.img = img
        self.y, self.x, _ = img.shape
        self.img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.h, self.s, self.v = cv2.split(self.img_hsv)

        #warp_img
        self.warp_img_size = [self.x,self.x]
        self.warp_img_zoomx = self.x//4

    def img_transform(self): # img_hsv -> img
        img_hsv = self.img_hsv

        ## 노란선 흰선 분리
        lower_yellow = np.array([15, 100, 140])
        upper_yellow = np.array([30, 200, 255])
        self.yellow_range = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 140])
        upper_white = np.array([50, 70, 255])
        self.white_range = cv2.inRange(img_hsv, lower_white, upper_white)

        ## 합치기
        self.combined_range = cv2.bitwise_or(self.yellow_range, self.white_range)
        # filtered_image = cv2.bitwise_and(self.cv_img, self.cv_img, mask = combined_range)

        self.img = self.combined_range
        
    def img_warp(self, img=None, change_img=True, warp_img_zoomx=None): # warp_img_size,warp_img_zoomx,img,x,y -> img,x,y
        y, x = self.y, self.x
        if img is None: img = self.img
        warp_img_size = self.warp_img_size
        if warp_img_zoomx is None:warp_img_zoomx = self.warp_img_zoomx

        ## Warp img to ROI(warped img)
        topx = 269
        bottomx = -25
        topy = 271
        bottomy = y
        src_points = np.float32([[bottomx, bottomy], [topx, topy], [x - topx, topy], [x-bottomx, bottomy]])
        # print(src_points)
        topx = warp_img_zoomx
        bottomx = warp_img_zoomx
        topy = x//4
        bottomy = x
        dst_points = np.float32([[bottomx, bottomy], [topx, topy], [x - topx, topy], [x-bottomx, bottomy]])
        # print(dst_points)
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        # print(matrix)

        self.warped_img = cv2.warpPerspective(img, matrix, warp_img_size)

        if change_img:
            self.img = self.warped_img
            self.x, self.y = warp_img_size

    def sliding_window(self): # nwindows,margin,minpix,threshold,midpoint,img -> l_lane,r_lane
        nwindows = self.nwindows
        margin = self.margin
        minpix = self.minpix
        lane = self.img
        midpoint = self.x//2
        threshold = self.threshold

        histogram = np.sum(lane, axis=0)

        leftx_current = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint

        window_height = np.int(self.x/nwindows)
        nz = lane.nonzero() # nz[0] is vertical(y), nz[1] is horizontal(x)

        left_lane_inds = []
        right_lane_inds = []
        
        lx, ly, rx, ry = [], [], [], []

        self.out_img = np.dstack((lane, lane, lane))*255

        l_err = [0,0]
        r_err = [0,0]
        foundr, foundl = (False, False)

        for window in range(nwindows):

            win_yl = self.x - (window+1)*window_height
            win_yh = self.x - window*window_height

            win_xll = leftx_current - margin
            win_xlh = leftx_current + margin
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            cv2.rectangle(self.out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
            cv2.rectangle(self.out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

            good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                if leftx_current > threshold and leftx_current < histogram.shape[0]-threshold:
                    l_err[1] = 0
                    foundl = True 
                else: l_err[1] += 1 
                leftx_current = np.int(np.mean(nz[1][good_left_inds]))
            else: l_err[1] += 1
            if not foundl: l_err[0] += 1
            if len(good_right_inds) > minpix:
                if rightx_current > threshold and rightx_current < histogram.shape[0]-threshold:
                    r_err[1] = 0
                    foundr = True
                else: r_err[1] += 1
                rightx_current = np.int(np.mean(nz[1][good_right_inds]))
            else: r_err[1] += 1
            if not foundr: r_err[0] += 1

            lx.append(leftx_current)
            ly.append((win_yl + win_yh)/2)

            rx.append(rightx_current)
            ry.append((win_yl + win_yh)/2)

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        self.out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        self.out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]

        self.l_lane, self.r_lane = (lx, ly, l_err), (rx, ry, r_err)

    ##판단
        
    def adjust_img(self):
        if self.cut_img_top:
            if np.sum(self.img[:][:self.y//2]) > 7000000:
                self.cut_img_top = None
                print("///////////")
            self.img[:][:self.y//2] = 0
        if self.yellow_based_slidingwindow:
            self.img_backup = self.img
            self.y, self.x = self.yellow_range.shape
            self.img_warp(img=self.yellow_range)

    def go_forward(self):
        poslf, posrf = self.l_lane, self.r_lane
        x = self.x
        line_len = 286

        posl = int(np.mean(poslf[0][1:4]))
        posr = int(np.mean(posrf[0][1:3]))
        fail_threshold = 3
        # print(posr[2])
        if max(poslf[2]) >= fail_threshold: posl = posr - line_len 

        if not self.trustr:
            if max(posrf[2]) >= fail_threshold: posr = posl + line_len

            if posl == posr:
                posl = x//2
        
        self.pos = (posl + posr) // 2
        
    def go_left(self, offset = 60):
        posl = self.l_lane
        posl = int(posl[0][3])
        x = self.x
        line_len = 286
        posl -= offset

        self.pos = (posl*2 + line_len)/2

    def go_right(self, offset = 60):
        if self.cut_img_top is not None: self.cut_img_top = True
        posl = self.r_lane
        posl = int(posl[0][3])
        x = self.x
        line_len = 286
        posl += offset

        self.pos = (posl*2 - line_len)/2
      
    def go_only_right(self, offset = 60):
        posl = self.r_lane
        posl = int(posl[0][3])
        x = self.x
        line_len = 286
        posl += offset

        self.pos = (posl*2 - line_len)/2
        if self.pos < x/2: self.pos = x//2

    def go_yellow(self):
        lane = np.sum(self.yellow_warped, axis=0)

        posl = 0
        for i, p_cur in enumerate(lane):
            posl += (i*p_cur)
        posl /= np.sum(lane)

        self.pos = (posl - 261) * 2 + self.x//2

    def go_yellow_obstacle(self):
        lane = np.sum(self.yellow_warped, axis=0)
        if self.current_lane == "RIGHT":
            lane_pos = 128 #148
            #self.current_lane = "LEFT"
            
        elif self.current_lane == "LEFT":
            lane_pos = 281 #261
            #self.current_lane = "RIGHT"
        #print(self.current_lane)
        posl = 0
        for i, p_cur in enumerate(lane):
            posl += (i*p_cur)
        posl /= np.sum(lane)

        self.pos = (posl - lane_pos) * 2 + self.x//2

    def go_sequence(self, current_position=None):
        if self.sequence == -1:
            print("seq -1")
            self.speed = 1500
            self.go_forward()
            self.start_time = time.time()
            self.sequence = -0.5
            self.midrange -= 70
            #3번째 지나고 출발
            # self.sequence = 12
            # self.speed = 500
        
        if self.sequence == -0.5:
            self.directControl = 0.5
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= 0.1:
                self.directControl = None
                self.sequence = 0

        elif self.sequence == 0:
            print("seq 0")
            # self.go_forward()
            self.obstacle_decide()
            elapsed_time = time.time() - self.start_time
            if True:
                self.start_time = time.time()
                self.sequence = 1
                self.yellowline_count = 0

        elif self.sequence == 1:
            print("seq 1")
            # self.go_forward()
            self.obstacle_decide()
            elapsed_time = time.time() - self.start_time
            #if self.yellowline_count >= 2:
            if self.is_reached_before_ratary(current_position):
                print("도착!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self.start_time = time.time()
                self.sequence = 2
                self.midrange += 100
            else:
                print("도착안함!!!!!!!!!!!!!!!!!!!!!!")
            

        elif self.sequence == 2:
            print("seq 2")
            self.speed = 400
            self.go_left()
            elapsed_time = time.time() - self.start_time
            if 8 <= elapsed_time:
                self.start_time = time.time()
                # self.directControl
                self.midrange -= 70
                self.sequence = 3
        
        elif self.sequence == 3:
            self.go_left()
            if max(self.r_lane[2]) == 0:
                self.sequence = 3.5
                
        elif self.sequence == 3.5:
            self.stopline_count = 0
            self.midrange += 70
            self.sequence = 4
        
        elif self.sequence == 4:
            print("seq 4")
            self.go_forward()
            if self.stopline_count >= 1:
                self.start_time = time.time()
                self.sequence = 5
            
        elif self.sequence == 5:
            print("seq 5")
            self.speed = 0
            elapsed_time = time.time() - self.start_time
            #앞에 차 없으면 우회전하는 코드 추가
            if (self.obstacle_state == "STRAIGHT_NEAR_N" or self.obstacle_state == "STRAIGHT_NEAR_F") and self.seq5_toggle == False:
                self.seq5_toggle = True
            if (self.obstacle_state == "STRAIGHT_NEAR_N" or self.obstacle_state == "STRAIGHT_NEAR_F")  and self.seq5_toggle == True:
                self.start_time = time.time()
                self.tmp_time = time.time()
                self.speed = 500*1.8
                self.sequence = 6
            
        elif self.sequence == 6:
            print("seq 6")
            
            x = 1.8
            if self.obstacle_state == "STRAIGHT_NEAR_N":
                self.speed = 0
                self.temp_elapsed_time = time.time() - self.tmp_time
            else:
                self.speed = 500*1.8
                else_time = time.time() - self.start_time
            elapsed_time = else_time - self.temp_elapsed_time
            if elapsed_time < 0.7/x:
                self.directControl = 0.5
            elif elapsed_time < 2/x:
                self.directControl = 1
            elif elapsed_time < 2.5/x:
                self.directControl = 0.8
            # elif elapsed_time < 3:
            #     self.directControl = 0.3
            # elif elapsed_time < 3.3:
            #     self.directControl = 0.3
            elif elapsed_time < 3.6/x:
                self.directControl = 0.3
            elif elapsed_time < 3.7/x:
                self.directControl = 0.35
            elif elapsed_time < 4.5/x:
                self.directControl = 1
            else:
                self.directControl = None
                self.sequence = 6.5

        elif self.sequence == 6.5:
            self.sequence = 7

        elif self.sequence == 7:
            print("seq 7")
            self.go_yellow()
            self.speed = 800
            print(np.sum(self.yellow_warped))
            if np.sum(self.yellow_warped) <= 400000:
                self.speed = 0
                self.sequence = 7.5
                
            
        elif self.sequence == 7.5:
            print("Traffic Light")
            if self.left_traffic:
                self.start_time = time.time()
                self.directControl = 0.45
                self.speed = 400
                self.stopline_count = 0
                self.sequence = 8

        elif self.sequence == 8:
            print("횡단보도 좌회전")
            elapsed_time = time.time() - float(self.start_time)
            if elapsed_time < 1:
                self.directControl = 0.3
            else:
                self.directControl = None
                self.go_left()
                if elapsed_time >= 5:
                    self.start_time = time.time()
                    self.sequence = 8.5
                
        elif self.sequence == 8.5:
            elapsed_time = time.time() - float(self.start_time)
            self.directControl = 0.3
            if elapsed_time >= 1.5:
                self.directControl = None
                self.start_time = time.time()
                self.sequence = 9
        
        elif self.sequence == 9:
            self.go_yellow()
            self.speed = 800
            if np.sum(self.yellow_warped) <= 300000:
                self.sequence = 10
                print("seq end")

        elif self.sequence == 10:
            # print("seq 10")
            self.speed = 400
            self.go_right()
            if self.cut_img_top == None:
                self.start_time = time.time()
                self.sequence = 10.5
                
        elif self.sequence == 10.5:
            elapsed_time = time.time() - float(self.start_time)
            self.directControl = 0.8
            if elapsed_time >= 1:
                self.directControl = None
                self.start_time = time.time()
                self.sequence = 11
        
        elif self.sequence == 11:
            self.speed = 1300
            print("seq 11")
            self.go_yellow()
            elapsed_time = time.time() - float(self.start_time)
            if max(self.r_lane[2]) == 0 and elapsed_time > 2:
                self.sequence = 12
                self.stopline_count = 0
        
        elif self.sequence == 12 or self.sequence == 12.5:
            print("seq 12")
            self.yellow_based_slidingwindow = True
            self.go_left(offset=0)
            if self.sequence == 12.5:
                self.img = self.img_backup
            self.sequence = 12.5
            if self.stopline_count >= 1:
                self.start_time = time.time()
                self.sequence = 13

        elif self.sequence == 13:
            print("seq 13")
            self.directControl = 0.5
            if np.sum(self.yellow_warped) > 400000:
                self.start_time = time.time()
                self.stopline_count = 0
                self.directControl = None
                self.sequence = 14

        elif self.sequence == 14:
            print("seq 14")
            self.go_yellow()
            elapsed_time = time.time() - float(self.start_time)
            if elapsed_time > 2:
                self.sequence = 15
                self.stopline_count = 0
        
        elif self.sequence == 15 or self.sequence == 15.5:
            print("seq 15")
            self.yellow_based_slidingwindow = True
            self.go_left(offset=0)
            if self.sequence == 15.5:
                self.img = self.img_backup
            self.sequence = 15.5
            if self.stopline_count >= 1:
                self.sequence = 16
                print("seq end")
                # self.speed = 0
    
    def yellow_lineDetect(self, lane = None):
        
        posl = np.sum(self.yellow_warped[:][50:-50])
        
        if posl < 200000: #sliding window view stop range
            if self.yellowline_toggle >= 2:
                self.yellowline_count += 1
                # print(self.stopline_count)
                self.yellowline_toggle = 0
        else: self.yellowline_toggle += 1

    def stop_line(self, lane = None):
        if lane is None: lane = self.img
        
        r_histogram = np.sum(lane, axis=1)
        r_histogram[r_histogram<55000] = 0 #히스토그램상 정지선 인식 범위
        posl = 0
        for i, p_cur in enumerate(r_histogram):
            posl += (i*p_cur)
        posl /= np.sum(r_histogram)
        if 550 < posl < 650: #sliding window view stop range
            if self.stopline_toggle >= 10:
                self.stopline_count += 1
                # print(self.stopline_count)
                self.stopline_toggle = 0
        else: self.stopline_toggle += 1

    
    def stop(self):
        self.speed = 0
        
    ############ new 
    
    def obstacle_decide(self):
        # rospy.loginfo("OBSTACLE DECIDE")
        if self.current_lane == "RIGHT":
            self.lane_msg.data = 1
        elif self.current_lane == "LEFT":
            self.lane_msg.data = 2
            
        ##################
        # if not self.is_safe :
        self.y_list_sort = sorted(self.y_list, key=lambda x: x)
        # rospy.loginfo("Y_LIST{}".format(self.y_list_sort))
        while len(self.y_list) > 5:
            del self.y_list[0]
        while len(self.y_list_sort) > 5:
            del self.y_list_sort[0]
        self.difference = self.y_list_sort[-1] - self.y_list_sort[0]
        #print("obsticle_sequence =", self.static_sequence)
        #print("diff =", self.difference)
        if self.static_sequence == 0:
            #print("in SEQUENCE IF", self.obstacle_state)
            if self.obstacle_state == "LEFT" or self.obstacle_state == "STRAIGHT_FAR":
                if self.difference >= 0.2 :
                    self.dynamic_flag = True
                    #rospy.loginfo("DYNAMIC OBSTACLE")
                    #rospy.loginfo("MISSION : DYNAMIC")
                    self.static_mission = False
                    self.speed = 0
                else :
                    #rospy.loginfo("NOT OBSTACLE !!!!!!")
                    self.directControl = None
                    self.speed = 800
                    self.go_forward()
            elif (self.obstacle_state == "STRAIGHT_NEAR_N" or self.obstacle_state == "STRAIGHT_NEAR_F") :
                if self.difference >= 0.2 :
                    self.dynamic_flag = True
                    #rospy.loginfo("DYNAMIC OBSTACLE")
                    #rospy.loginfo("MISSION : DYNAMIC")
                    self.static_mission = False
                    self.speed = 0
                else :
                    self.speed = 800
                    self.static_flag += 1
                    if self.static_flag >= 1: self.static_sequence = 1
            else: 
                #rospy.loginfo("NOT OBSTACLE !!!!!!")
                self.directControl = None
                self.speed = 1100
                self.go_forward()

        elif self.static_sequence == 1:
            self.directControl = None
            self.speed = 600
            if self.current_lane == "LEFT":
                self.current_lane = "RIGHT"

            elif self.current_lane == "RIGHT":
                self.current_lane = "LEFT"
            self.obstacle_t1 = rospy.get_time()
            self.static_sequence = 1.5
            # self.midrange -= 70
            
        elif self.static_sequence == 1.5:
            self.directControl = 0
            t2 = rospy.get_time() - self.obstacle_t1
            if t2 > 0.5:
                self.directControl = None
                self.static_sequence = 2
            

        elif self.static_sequence == 2:
            t2 = rospy.get_time() - self.obstacle_t1
            
            self.go_yellow_obstacle()
            if isnan(self.pos): self.go_only_right(offset=0)
            if t2 > 1.5 and self.obstacle_state != "RIGHT":
                self.current_lane = "RIGHT"
                self.static_sequence = 2.5
                self.obstacle_t1 = rospy.get_time()
                
        elif self.static_sequence == 2.5:
            self.directControl = 1
            t2 = rospy.get_time() - self.obstacle_t1
            if (self.obstacle_state == "STRAIGHT_NEAR_N" or self.obstacle_state == "STRAIGHT_NEAR_F") and t2 >= 0.6:
                self.static_sequence = 0
                # self.speed = 0
                # time.sleep(0.2)
                # self.midrange += 70
            if t2 > 0.61:
                self.directControl = None
                self.current_lane = "RIGHT"
                self.static_sequence = 3
                # self.midrange += 70

        elif self.static_sequence == 3:
            t2 = rospy.get_time() - self.obstacle_t1
            self.go_yellow_obstacle()
            if isnan(self.pos): self.go_only_right(offset=0)
            if (self.obstacle_state == "STRAIGHT_NEAR_N" or self.obstacle_state == "STRAIGHT_NEAR_F") and t2 >= 1:
                self.static_sequence = 0
            if t2 > 3:
                self.static_sequence = 0
    # # def lane_change
    # ####################### obstalce detector ########################
    # def obstacle_decide(self):
    #     # #rospy.loginfo("OBSTACLE DECIDE")
    #     if self.current_lane == "RIGHT":
    #         self.lane_msg.data = 1
    #     elif self.current_lane == "LEFT":
    #         self.lane_msg.data = 2
    #     ##################
    #     if not self.is_safe :
    #         self.y_list_sort = sorted(self.y_list, key=lambda x: x)
    #         #rospy.loginfo("Y_LIST{}".format(self.y_list_sort))
            
    #         if len(self.y_list) <= 15 :
    #             self.stop()
            
    #         else :
    #             while len(self.y_list) > 15:
    #                 del self.y_list[0]
    #             while len(self.y_list_sort) > 15:
    #                 del self.y_list_sort[0]
            
    #         obstacle_t1 = rospy.get_time()
            
    #         if self.obstacle_state == "STRAIGHT" :
    #             self.difference = self.y_list_sort[-1] - self.y_list_sort[0]
    #             if self.difference >= 8 :
    #                 self.dynamic_flag = True
    #                 #rospy.loginfo("DYNAMIC OBSTACLE")
    #                 self.dynamic_obstacle_drive()
                    
                    
    #             else :
    #                 self.static_flag = True
    #                 #rospy.loginfo("STATIC OBSTACLE")
    #                 self.static_obstacle_drive()
                    
                    
    #             # self.static_flag = True
    #             # #rospy.loginfo("STATIC OBSTACLE")
    #             #self.static_obstacle_drive()
    #         # elif self.obstacle_state == "LEFT":
    #         #     # obstacle_t2 = rospy.get_time()
    #         #     # if obstacle_t2 - obstacle_t1 <= 5 :
    #         #     if self.obstacle_state == "STRAIGHT" or self.obstacle_state == "RIGHT" : 
    #         #         self.dynamic_flag = True
    #         #         rospy.logwarn("DYNAMIC OBSTACLE")
    #         #         self.dynamic_obstacle_drive()
            

    #         # elif self.obstacle_state == "RIGTH":
                
        
    #         # elif self.obstacle_state == "LEFT":
                
    #         # obstacle_t1 = rospy.get_time()
    #         # if self.obstacle_state == "LEFT" :
    #         #     #rospy.loginfo("plzzzzzzzzzzzzzzzzzzzzzzzzz")
    #         #     obstacle_t2 = rospy.get_time()
    #         #     time.sleep(0.7)
    #         #     if obstacle_t2 - obstacle_t1 <= 5 :
    #         #         if self.obstacle_state == "STRAIGHT" or self.obstacle_state == "RIGHT" : 
    #         #             self.dynamic_flag = True
    #         #             rospy.logwarn("DYNAMIC OBSTACLE")
    #         #             self.dynamic_obstacle_drive()
    #         #         else : 
    #         #             rospy.logwarn("설마?????????????????????")
                        
    #         #             # self.static_flag = True ################################참고
    #         #     else:
    #         #         print("시간!!!", obstacle_t2 - obstacle_t1)
                        
    #         # elif self.obstacle_state == "STRAIGHT" : 
    #         #     self.static_flag = True
    #         #     #rospy.loginfo("STATIC OBSTACLE")
    #         #     self.static_obstacle_drive()
    #         # else :
    #         #     pass
    #         else :
    #             #rospy.loginfo("그냥 주행!!!!!!!!")
    #             self.directControl = None
    #             self.current_lane = "RIGHT"
    #             # self.is_safe = True
    #             self.static_flag = False
    #             self.dynamic_flag = False
    #             self.speed = 500
    #             self.go_forward()
    #             # self.just_drive()
            
                
    #     else:
    #         #rospy.loginfo("NOT OBSTACLE !!!!!!")
    #         self.directControl = None
    #         self.speed = 1000
    #         self.go_forward()
    #         # self.just_drive()
    #         while len(self.y_list) > 10:
    #             del self.y_list[0]
    #         while len(self.y_list_sort) > 10:
    #             del self.y_list_sort[0]
           
                            
        # # dynamic_obstacle_drive
        # if self.dynamic_flag and not self.is_safe:
        #     rospy.logwarn("DYNAMIC OBSTACLE")
        #     # self.stop()
        #     self.dynamic_obstacle_drive()

        # # static_obstacle_drive
        # elif self.static_flag and not self.is_safe :  # true
        #     #rospy.loginfo("STATIC OBSTACLE")
        #     self.static_obstacle_drive()
        


            
            
            
    def just_drive(self):
        self.current_lane = "RIGHT"
        self.go_yellow_obstacle()
            
        # static obstacle drive
    def static_obstacle_drive(self):
        # 왼쪽으로 각도 틀고 원래 차선으로 복귀 후 주행
        #rospy.loginfo("MISSION : STATIC")
        t2 = rospy.get_time()
        if self.static_flag:
            self.static_t1 = rospy.get_time()
            self.static_mission = True
            self.static_flag = True
            
        self.speed = 500
        self.current_lane = "LEFT"

        # if self.current_lane == "RIGHT":
        #    if t2 - self.static_t1 < 2.0 : #몇초동안 멈출지
        #        self.speed_msg.data = 0
        #    else :
        #        if static_flag

        # if self.current_lane == "RIGHT" and not self.changed_lane_toggle:
        #     self.speed = 500
        #     self.current_lane = "LEFT"
        #     self.changed_lane_toggle = True
                

        # elif self.current_lane == "LEFT" and not self.changed_lane_toggle:
        #     self.speed = 500
        #     self.current_lane = "RIGHT"
        #     self.changed_lane_toggle = True
            
        
        self.go_yellow_obstacle()

    # dynamic obstacle drive
    def dynamic_obstacle_drive(self):
        # 멈췄다가 주행
        #rospy.loginfo("MISSION : DYNAMIC")
        self.static_mission = False
        self.dynamic_mission = True
        self.speed = 0
        

                
            
        ##############################################    
        
    ##################### 주 석 처 리 ##########################################

    #     if not self.is_safe:

    #         self.y_list_sort = sorted(self.y_list, key=lambda x: x)
    #         #rospy.loginfo("Y_LIST{}".format(self.y_list))

    #         # dynamic_obstacle_drive
    #         if len(self.y_list) <= 19:
    #             self.stop()
    #             self.go_forward()
    #             # #rospy.loginfo("obstacle_stop, dynamic_flag = {}", format(self.dynamic_flag))

    #         # dynamic ostacle loginfo
    #         elif abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])) >= 0.17 or \
    #             self.y_list_sort[10] < -0.15:
    #             self.dynamic_flag = True
    #         #     self.static_flag = False
    #             # self.y_list.clear
            
    #             # #rospy.loginfo("dynamic")
    #             # #rospy.loginfo(self.y_list_sort)
    #             # #rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

    #         # static obstacle loginfo
    #         else:
    #             self.static_cnt += 1
    #         #     if self.static_cnt >= 10:
    #         #         self.static_flag = True
    #         #         self.dynamic_flag = False
    #         #         self.static_cnt = 0
    #             # #rospy.loginfo("static")
    #             # #rospy.loginfo(self.y_list_sort)
    #             # #rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

    #         # dynamic_obstacle_drive
    #         if self.dynamic_flag and not self.is_safe:
    #             rospy.logwarn("DYNAMIC OBSTACLE")
    #             # self.stop()
    #             self.dynamic_obstacle_drive()

    #         # static_obstacle_drive
    #         elif self.static_flag:  # true
    #             #rospy.loginfo("STATIC OBSTACLE")
    #             self.static_obstacle_drive()
    #         # if the car is driving depending on "right" window
# .car.yellow_warped)
       
    #         self.directControl = None
    #         self.go_forward()

    # # static obstacle drive
    # def static_obstacle_drive(self):
    #     # 왼쪽으로 각도 틀고 원래 차선으로 복귀 후 주행
    #     #rospy.loginfo("MISSION : STATIC")
    #     t2 = rospy.get_time()
    #     if not self.static_flag:
    #         self.static_t1 = rospy.get_time()
    #         self.static_mission = True
    #         self.static_flag = True

    #     # if self.current_lane == "RIGHT":
    #     #    if t2 - self.static_t1 < 2.0 : #몇초동안 멈출지
    #     #        self.speed_msg.data = 0
    #     #    else :
    #     #        if static_flag

    #     if self.current_lane == "LEFT":
    #         self.speed = 1000
    #         self.current_lane = "RIGHT"
    #         self.go_yellow_obstacle()
                

    #     elif self.current_lane == "RIGHT":
    #         self.speed = 1000
    #         self.current_lane = "LEFT"
    #         self.go_yellow_obstacle()

    # # dynamic obstacle drive
    # def dynamic_obstacle_drive(self):
    #     # 멈췄다가 주행
    #     rospy.loginfo("MISSION : DYNAMIC")
    #     self.static_mission = False
    #     self.speed = 0
    #     self.directControl = 0.5
    
    
    ###########################################

    ##제어
    def control_pub(self, ctrl = None): # speed,midrange,x,pos
        midrange = self.midrange
        x = self.x
        pos = self.pos
        
        # pid = round(self.pidcal.pid_control(pos, PART=None, setpoint=x // 2), 6)
        # self.cmd_msg.data = 0.5 - pid
   
        midstart = x//2-midrange
        midend = x//2+midrange
        if ctrl is None:
            if not isnan(pos):
                ctrl = (((pos - midstart) * (1 - 0)) / (midend - midstart)) + 0
            else:
                ctrl = 0.5 # ctrl = self.cam.keycode
                print("pos is nan")
        else: print("directControl")
        self.steering_msg.data = ctrl #조향각 설정
        
        self.speed_msg.data = self.speed

    ## 테스트용 메서드
    def use_test_img(self, img_src = "src/lane_detection/scripts/img.jpg"): # -> cv_img
        self.cv_img = cv2.imread(img_src, cv2.IMREAD_UNCHANGED)
    def print_img_hsv(self): # img,h,s,v
        img = self.img
        h, s, v = self.h, self.s, self.v

        cv2.namedWindow("h", cv2.WINDOW_NORMAL)
        cv2.namedWindow("s", cv2.WINDOW_NORMAL)
        cv2.namedWindow("v", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

        cv2.imshow("h", h)
        cv2.imshow("s", s)
        cv2.imshow("v", v)
        cv2.imshow("Image", img)
        
    def is_reached_before_ratary(self, current_position):
        #================= 교차로 들어가기 전 웨이포인트 =========================
        waypoint_rotary = MoveBaseGoal()
        waypoint_rotary.target_pose.header.frame_id = "map"
        waypoint_rotary.target_pose.pose.position.x = 31.404663415273333
        waypoint_rotary.target_pose.pose.position.y = 1.0858890951974947
        waypoint_rotary.target_pose.pose.orientation.w = 0.002997711827324864
        waypoint_rotary.target_pose.pose.orientation.z = -0.9999955068518059
        #=======================================================================
        
        if current_position is None:
            return False
        
        distance = math.sqrt((current_position.pose.pose.position.x - waypoint_rotary.target_pose.pose.position.x) ** 2 + \
        (current_position.pose.pose.position.y - waypoint_rotary.target_pose.pose.position.y) ** 2)
        
        if abs(distance) < 0.5:
            return True
        
        return False

    ## ROS 관련 메서드
    # def subscribe(self):
    #     rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self._img_cb, queue_size=1)
    # def _img_cb(self, msg):
    #     self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)

    # def sub_traffic(self):
    #     rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, callback=self._traffic_cb, queue_size=1)
    # def _traffic_cb(self, msg):
    #     if msg.trafficLightStatus == 33:
    #         self.left_traffic = True
    #     else:
    #         # print("Status:", msg.trafficLightStatus)
    #         self.left_traffic = False

        
    

# if __name__ == "__main__":
#     car = LaneFollower()
#     car.subscribe()
#     car.sub_traffic()
#     while not rospy.is_shutdown():
#         while car.cv_img is None: car.rate.sleep()
#         #인지
#         car.img_init(car.cv_img)
#         car.img_transform()
#         car.img_warp(car.yellow_range, change_img=False, warp_img_zoomx=car.x//2.5)
#         car.yellow_warped = car.warped_img
#         car.img_warp()
#         car.adjust_img()#판단
#         car.sliding_window()

#         #판단
#         car.go_sequence()
#         car.stop_line()

#         #제어
#         car.control_pub(ctrl=car.directControl)
        
#         #cv2.imshow("img", car.cv_img)
#         cv2.circle(car.out_img, (int(car.pos), 550), 5, (255, 255, 255))
#         cv2.imshow("lane", car.out_img)
#         cv2.waitKey(1)
#         car.rate.sleep()