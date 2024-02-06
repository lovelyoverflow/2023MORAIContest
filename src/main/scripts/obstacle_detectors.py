#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import math
import time
from std_msgs.msg import String, Int32, Float32
from obstacle_detector.msg import Obstacles


class LidarReceiver():

    def __init__(self):
        rospy.loginfo("00000lidar callback")
        
        # subscriber: raw_obstacles를 받아 장애물을 인식함
        rospy.Subscriber("/raw_obstacles", Obstacles, self.lidar_callback)
        # publisher: 장애물 인식 상태를 전달함
        self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5)
        self.object_pub = rospy.Publisher("object_condition", Float32, queue_size=5)

        self.obstacle_pos_pub = rospy.Publisher("obstacle_pos", String, queue_size=5)

        self.count_flag = 0
        self.flag_flag = 0
        self.count_t1 = 0
        self.x1 = 0
        self.x2 = 0
        
        ##########
        self.obstacle_state = String()
        self.ylist = Float32()
        #############


    def lidar_callback(self, _data):
        WARNING_CNT = 1

        self.point_cnt = 0   # ROI 내 장애물 개수\

        data_sorted = sorted(_data.circles, key=lambda sort: sort.center.x)
        while len(data_sorted) != 0 and data_sorted[0].center.x < 0:
            # print("DELETING", data_sorted[0])
            del data_sorted[0]
        
        while len(data_sorted) != 0 and data_sorted[0].center.y > 2:
            # print("DELETING", data_sorted[0])
            del data_sorted[0]
            

        if len(data_sorted) == 0: 
            rospy.loginfo("NONE")
            self.obstacle_state.data = "NONE"
            self.point_cnt += 1
        else:
            # print("USING", data_sorted[0])
            i = data_sorted[0]

            #print("y=", i.center.y, " x=", i.center.x)
            
            
            self.point_cnt += 1
            self.ylist.data = i.center.y

            if i.center.y < -0.18:
                if i.center.x < 1: #2.6:
                    if i.center.x >= 0.05:
                        self.obstacle_state.data = "RIGHT"
                        rospy.loginfo("RIGHT")
                        
            elif abs(i.center.y) < 0.18:
                if i.center.x < 2.6 and i.center.x > 1.2:
                    self.obstacle_state.data = "STRAIGHT_FAR"
                    rospy.loginfo("STRAIGHT_FAR")
                elif i.center.x < 1.2: #2.6:
                    if i.center.x >= 0.05:
                        self.obstacle_state.data = "STRAIGHT_NEAR"
                        rospy.loginfo("STRAIGHT_NEAR")
                    
            else:
                if i.center.x >= 0.05:
                    self.obstacle_state.data = "LEFT"
                    rospy.loginfo("LEFT")

        if self.point_cnt >= WARNING_CNT:
            # rospy.loginfo("lidar callback warning")
            self.warning_pub.publish("WARNING")
            self.obstacle_pos_pub.publish(self.obstacle_state)
            self.object_pub.publish(self.ylist)
            self.point_cnt = 0
            
        # 장애물이 일정 개수 미만이면 safe 상태
        else:
            self.warning_pub.publish("safe")
            self.point_cnt = 0
            self.object_pub.publish(self.ylist)


def run():
    try:
        rospy.init_node("lidar_obstacle_detector")
        new_class = LidarReceiver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    run()