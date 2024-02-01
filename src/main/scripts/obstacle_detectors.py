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

        self.count_flag = 0
        self.flag_flag = 0
        self.count_t1 = 0
        self.x1 = 0
        self.x2 = 0


    def lidar_callback(self, _data):
        rospy.loginfo("lidar callback")
        
        # ROI: 장애물 감지 영역 설정
        left_y = -5  # 차량 왼쪽 0.2 m
        right_y = 2  # 차량 오른쪽 0.2 m
        front_x = 3 # 차량 앞 1.6 m
        back_x = 10     # 차량 뒤 0 m
        WARNING_CNT = 1

        self.point_cnt = 0   # ROI 내 장애물 개수
        self.dynamic_cnt = 0 # 

        for i in _data.circles:
            # ROI 내의 장애물 위치
            # rospy.loginfo("lidar callback2")
            print(i.center.y)
            if left_y < i.center.y < right_y and front_x < i.center.x < back_x:
                # rospy.loginfo("lidar callback3")
                self.object_pub.publish(i.center.y)
                # rospy.loginfo("lidar XXXXXXXXXXXX")
                
            if left_y < i.center.y < right_y and front_x < i.center.x < back_x:
                # rospy.loginfo("lidar callback4")
                self.point_cnt += 1

            # 장애물이 일정 개수 이상 나타나면 WARNING 상태
            if self.point_cnt >= WARNING_CNT:
                # rospy.loginfo("lidar callback warning")
                self.warning_pub.publish("WARNING")
            # 장애물이 일정 개수 미만이면 safe 상태
            else:
                self.warning_pub.publish("safe")
                self.point_cnt = 0


def run():
    try:
        rospy.init_node("lidar_obstacle_detector")
        new_class = LidarReceiver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    run()