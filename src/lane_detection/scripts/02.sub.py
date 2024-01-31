#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32

def CB(msg):
    print(msg.data*2)

rospy.init_node("wego_sub_node") #1. node 이름 설정
rospy.Subscriber("counter", Int32, callback=CB)
rospy.spin()