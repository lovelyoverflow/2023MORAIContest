#!/usr/bin/env python3
import rospy
import os, rospkg
from time import sleep, time

import numpy as np
from cv_bridge import CvBridge
import cv2

from math import isnan
import statistics

# from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import CompressedImage

#from lane_detection.scripts.main_lane_detector import *
from warper import Warper
from slidingwindow import SlidingWindow
from rotary import Rotary
class MainController:
    def __init__(self) -> None:
        rospy.init_node("main_controller_run")
        
        self.warper = Warper()
        self.slidingwindow = SlidingWindow()
        #self.lane_detection = Cam_sub()
        self.bridge = CvBridge()
        self.rotary = Rotary()
        
        self.current_lane = "RIGHT"
        self.is_safe = True
        self.initialized = False

        turn = 0.2

        self.lmode = False
        self.trustr = True
        self.stopline_toggle = 0
        self.start_time = None
        self.start_time_breaker = False
        self.lmode_break = False

        
        self.lane_msg = Float64() # current_lane msg create
        self.speed_msg = Float64()
        self.angle_msg = Float64()
        
        self.initDriveFlag = True
        
        # dynamic obstacle mission
        self.dynamic_flag = False
        self.dynamic_mission = False 
        self.DynamicDrive = False
                
        # static obstacle mission
        self.static_flag = False
        self.static_mission = False
        self.turn_left_flag = 0
        self.turn_right_flag = 0
        
        # lidar warning string 
        self.lidar_warning = ""
        
        self.y_list = []
        self.y_list_sort = []

        # publisher 
        # rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        self.current_lane_pub = rospy.Publisher("/current_lane", Float64, queue_size=1) # 1 = right, 2 = left
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) 
        self.angle_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)

        # subscriber 
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self._img_cb)
        rospy.Subscriber("/lidar_warning", String, self.lidar_warning_callback)
        rospy.Subscriber("/object_condition", Float32, self.object_callback)

        self.initDrive_t1 = rospy.get_time()
        print(self.initDrive_t1)

        # self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # try:
                # self.timer_callback()
            self.main_drive()
                
            rospy.sleep(0.1)
            # rate.sleep()
            # except rospy.ROSInitException:
            #     pass
 
            
            # rate.sleep()
    
    # def timer_callback(self):
    #     # try:
    #     self.main_drive()
    #     rospy.loginfo("timer")
            # pass
        # except Exception as e:
        #     pass
    
    
    def _img_cb(self, msg):
        self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # self.laneColor(self.cv2_img)
        img_hsv = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2HSV)
        
        lower_lane = np.array([0, 0, 140])
        upper_lane = np.array([50, 70, 255])
        lane_range = cv2.inRange(img_hsv, lower_lane, upper_lane)
        
        lower_yellow = np.array([15, 100, 140])
        upper_yellow = np.array([30, 200, 255])
        yellow_range = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    
        combined_range = cv2.bitwise_or(yellow_range, lane_range)
        filtered_image = cv2.bitwise_and(self.cv2_img, self.cv2_img, mask = combined_range)
        
        self.laneDetection(combined_range)
        

    # def laneColor(self, img):
    #     self.cv2_img = img
    #     img_hsv = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2HSV)
        
    #     lower_lane = np.array([0, 0, 140])
    #     upper_lane = np.array([50, 70, 255])
    #     lane_range = cv2.inRange(img_hsv, lower_lane, upper_lane)
        
    #     lower_yellow = np.array([15, 100, 140])
    #     upper_yellow = np.array([30, 200, 255])
    #     yellow_range = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    
    #     combined_range = cv2.bitwise_or(yellow_range, lane_range)
    #     filtered_image = cv2.bitwise_and(self.cv2_img, self.cv2_img, mask = combined_range)
        
    #     self.laneDetection(combined_range)

        
    def laneDetection(self, combined_range):
        warped_img, self.x = self.warper.warp(combined_range)
        out_img, posl, posr , (posl_fail, posr_fail) = self.slidingwindow.slidingwindow(warped_img)
        rospy.loginfo("CVWWWWWWWWW")
        # self.rotary.rotary(warped_img, posr_fail)
        
        posl = int(posl[0][3])
        posr = int(posr[0][3])
        # print(posl-posr)
        line_len = 286
        fail_threshold = 3
        print(posr_fail)
        if self.lmode:
            posl -= 60
            posr = posl + line_len
            if not self.lmode_break:
                self.speed_msg = 400
                self.lmode_break= True
                self.speed_pub.publish(self.speed_msg)
            print("LMODE")
        # print(posl_fail)
        #if self.lmode:
        ##    self.kbd.motor_spdw = 400
        #    posl -= 60
        #    posr = posl + line_len
        #    # print("LMODE")
        
        
        if max(posl_fail) >= fail_threshold: posl = posr - line_len 

        if not self.trustr:
            if max(posr_fail) >= fail_threshold: posr = posl + line_len

            if posl == posr:
                posl = self.x//2

        cv2.imshow("sliding", out_img)
        cv2.waitKey(1)

        # keyCode = cv2.waitKey(1)
        # keycode = chr(keyCode & 0xFF)
        # if keycode == "l":
        #     self.lmode = not self.lmode
        # if keycode == "r":
        #     self.kbd.motor_spdw = self.speed

        # start of ctrl stuff 조향각(0~1) 코드 - 차선 변환 0~480
        midrange =220 #170 based
        midstart = self.x//2-midrange
        midend = self.x//2+midrange
        mid = self.x//2
        pos = (posl + posr) // 2
        if not isnan(posr):
            self.angle_msg.data = (((pos - midstart) * (1 - 0)) / (midend - midstart)) + 0
        else: 
            self.angle_msg.data = 0.5 # ctrl = self.cam.keycode
            
        # self.angle_msg.data = 0.5
        self.angle_pub.publish(self.angle_msg)
        
        # self.kbd.steer(ctrl)
        
    def lidar_warning_callback(self, msg):
        self.lidar_warning = msg.data
        if self.lidar_warning == "safe":
            self.is_safe = True
            
            # if self.static_mission:
            #     rospy.loginfo("??????????????????????/")
            #     self.is_safe = False
            # else:
            
        elif self.lidar_warning == "WARNING":
            self.is_safe = False
            rospy.loginfo("WARNING !! ")
        else:
            # rospy.loginfo("엥?????????????????????//")   
            pass
            
        
    def object_callback(self, msg):
        if self.dynamic_flag != True or len(self.y_list) <= 19:
            self.y_list.append(msg.data)
            if len(self.y_list) >= 21:
                del self.y_list[0]
        else:
            rospy.logwarn("Unknown warning state!")

    def static_obstacle_drive(self):
        # 왼쪽으로 각도 틀고 원래 차선으로 복귀 후 주행
        rospy.loginfo("MISSION : STATIC")
        t2 = rospy.get_time()
        
        if self.static_flag == False:
            self.static_t1 = rospy.get_time()
            self.static_mission = True
            self.static_flag = True
        
        #if self.current_lane == "RIGHT":
        #    if t2 - self.static_t1 < 2.0 : #몇초동안 멈출지
        #        self.speed_msg.data = 0
        #    else :
        #        if static_flag
                
        if self.current_lane == "LEFT":
            self.speed_msg.data = 1000
            if t2 - self.static_t1 < 1.8:
                self.angle_msg.data = 0.87
            elif t2 - self.static_t1 < 2.6:
                self.angle_msg.data = 0.27
            else:
                self.angle_msg.data = 0.57
                self.current_lane = "RIGHT"
                self.static_mission = False
                self.static_flag = False

        elif self.current_lane == "RIGHT":
            self.speed_msg.data = 1000
            if t2 - self.static_t1 < 1.8:
                self.angle_msg.data = 0.27
            elif t2 - self.static_t1 < 2.6:
                self.angle_msg.data = 0.87
            else:
                self.angle_msg.data = 0.57
                self.current_lane = "LEFT"
                self.static_mission = False
                self.static_flag = False
        
        self.speed_pub.publish(self.speed_msg)
        self.angle_pub.publish(self.angle_msg)

        
    def dynamic_obstacle_drive(self):
        # 멈췄다가 주행
        rospy.loginfo("MISSION : DYNAMIC")
        self.static_mission = False
        self.speed_msg.data = 0
        self.angle_msg.data = 0.5

        self.speed_pub.publish(self.speed_msg)
        self.angle_pub.publish(self.angle_msg)

    def default_drive(self):
        # self.angle_msg = Float64()
        rospy.loginfo("Default")
        self.speed_msg.data = 1000
        self.angle_msg.data = 0.5
        self.angle_pub.publish(self.angle_msg)
        self.speed_pub.publish(self.speed_msg)
        
    def init_drive(self):
        rospy.loginfo("initDrive")
        self.speed_msg.data = 1000
        self.angle_msg.data = 0.5
        self.initDrive_t2 = rospy.get_time()
        if self.initDrive_t2 - self.initDrive_t1 >= 0.01:
            rospy.loginfo('initinit')
            self.initDriveFlag = False
            rospy.loginfo('initinit22222222222222')
            
        else:
            print(self.initDrive_t2)
            # print(self.initDrive_t1)
            rospy.loginfo("time error")
        
        
        self.angle_pub.publish(self.angle_msg)
        self.speed_pub.publish(self.speed_msg)
        rospy.loginfo('initinit END')


    def main_drive(self):
        rospy.loginfo("start")
        if self.current_lane == "RIGHT":
            self.lane_msg.data = 1
        elif self.current_lane == "LEFT":
            self.lane_msg.data = 2
        self.current_lane_pub.publish(self.lane_msg)

        rospy.loginfo("MAIN")

        # 초기 주행
        if self.initDriveFlag:
            self.init_drive()

        # 동적 장애물
        elif not self.is_safe:
            
            self.y_list_sort = sorted(self.y_list, key=lambda x: x)
            rospy.loginfo("Y_LIST{}".format(self.y_list))

            # dynamic_obstacle_drive 
            if len(self.y_list) <= 19:
                # self.stop()
                self.dynamic_obstacle_drive()
                # rospy.loginfo("obstacle_stop, dynamic_flag = {}", format(self.dynamic_flag))

            # dynamic ostacle loginfo
            elif abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])) >= 0.17 or \
                    self.y_list_sort[10] < -0.15:
                self.dynamic_flag = True
                self.static_flag = False
                # self.y_list.clear
                rospy.loginfo("dynamic")
                # rospy.loginfo(self.y_list_sort)
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))
            
            # static obstacle loginfo 
            else:
                self.static_cnt += 1
                if self.static_cnt >= 10:
                    self.static_flag = True
                    self.dynamic_flag = False
                    self.static_cnt = 0
                rospy.loginfo("static")
                # rospy.loginfo(self.y_list_sort)
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

            # dynamic_obstacle_drive
            if self.dynamic_flag == True and self.is_safe == False:
                rospy.logwarn("DYNAMIC OBSTACLE")
                # self.stop()
                self.dynamic_obstacle_drive()

            # static_obstacle_drive
            elif self.static_flag: # true
                rospy.loginfo("STATIC OBSTACLE")
                self.static_obstacle_drive()
            # if the car is driving depending on "right" window

        else:
            rospy.loginfo("defualt drive")
            self.default_drive()

            
    #     rospy.loginfo("start")
    #     if self.current_lane == "RIGHT":
    #         self.lane_msg.data = 2
    #     elif self.current_lane == "LEFT":
    #         self.lane_msg.data = 1
    
    #     self.current_lane_pub.publish(self.lane_msg)

    #     rospy.loginfo("MAIN")

    #     # 초기 주행
    #     if self.initDriveFlag:
    #         self.initDrive()
    #         rospy.loginfo("next")

    # #############Publish
    #     # 동적 장애물
    #     elif self.is_safe == False and self.DynamicDrive == True:
    #         rospy.loginfo("dynamic")
    #         self.dynamic_obstacle_drive()

    #     # 정적 장애물
    #     elif self.is_safe == False :
    #         rospy.loginfo("statoc")
    #         self.static_obstacle_drive()
    #     else:
    #         rospy.loginfo("else")
    #         # self.defaultDrive()
    
        
def nothing(x):
    pass


if __name__ == "__main__":
    try:
        controller = MainController()
        # rospy.spin()
        
    except rospy.ROSInternalException:
        pass