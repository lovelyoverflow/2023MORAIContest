#!/usr/bin/env python3
#-*- coding:utf-8 -*-


import rospy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import math
import numpy as np
import matplotlib.pyplot as plt

X = np.zeros(1)
Y = np.zeros(1)

class lidar_sub():
    def __init__(self) -> None:
        rospy.init_node("sim_lidar_sub_node")
        rospy.Subscriber("/lidar2D", LaserScan, self._lidar_cb)
        self.image_msg = LaserScan()
        self.bridge = CvBridge()

    
    def _lidar_cb(self, msg):
        self.scan_msg = msg
        deg_min = self.scan_msg.angle_min
        deg_max = self.scan_msg.angle_max
        
        global X
        global Y

        X = np.linspace(deg_min,deg_max,len(self.scan_msg.ranges))
        Y = self.scan_msg.ranges

        for i, j in zip(X, Y):
            print(i)
            if 0.9*math.pi < i < 1.1*math.pi and j < 3:
                print("obsticle", np.argmin(Y))

        """ print(msg)
        print(deg_min)
        print(deg_max) """

fig, ax = plt.subplots()

plt.axes(polar=True)
line, = plt.plot([0], [10])
def update(frame):
    #print(Y)
    line.set_data(X, Y)
    return line,
from matplotlib.animation import FuncAnimation

def main():
    try:
        s = lidar_sub()
        
        ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128))
        plt.show()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()