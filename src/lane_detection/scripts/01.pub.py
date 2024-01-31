#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub_node") #node 이름 설정
pub = rospy.Publisher("/counter", Int32, queue_size=1) #node의 역할 설정
int_msg = Int32() #pub할 msg 인스턴스 만들기
rate = rospy.Rate(10) #Hz Ctrl용 인스턴스

num = 0
while not rospy.is_shutdown():
    num += 1
    int_msg.data = num
    pub.publish(int_msg) #여기서 pub
    print(num)
    rate.sleep() # 얘가 rate 조절