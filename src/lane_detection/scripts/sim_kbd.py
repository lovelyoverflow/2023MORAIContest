#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64
from KeyPoller import KeyPoller

class Sim_kbd():
    def __init__(self, motor_spdw = 1000.0, motor_spds = -2400.0, motor_spdn = 0.0, servo_l = 0.4, servo_r = 0.6, servo_c = 0.5) -> None:
        # rospy.init_node("sim_cmd_node")
        self.pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.pub_steer = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.cmd_msg = Float64()
        self.motor_spdw = motor_spdw
        self.motor_spds = motor_spds
        self.motor_spdn = motor_spdn
        self.servo_r = servo_r
        self.servo_l = servo_l
        self.servo_c = servo_c
        self.rate = rospy.Rate(30)
    
    def ctrl_keyb(self, c):
        # c = self.poll()
        if not c is None:
            if c == "w":
                self.cmd_msg.data = self.motor_spdw
                self.pub.publish(self.cmd_msg.data)
            elif c == "s":
                self.cmd_msg.data = self.motor_spds
                self.pub.publish(self.cmd_msg.data)
            elif c == "a":
                self.cmd_msg.data = self.servo_l
                self.pub_steer.publish(self.cmd_msg.data)
                self.cmd_msg.data = self.motor_spdw
                self.pub.publish(self.cmd_msg.data)
            elif c == "d":
                self.cmd_msg.data = self.servo_r
                self.pub_steer.publish(self.cmd_msg.data)
                self.cmd_msg.data = self.motor_spdw
            else:
                self.cmd_msg.data = self.motor_spdn
                self.pub.publish(self.cmd_msg.data)

                self.cmd_msg.data = self.servo_c
                self.pub_steer.publish(self.cmd_msg.data)
            # print(c)

    def steer(self, val):
        self.cmd_msg.data = val #조향각 설정
        self.pub_steer.publish(self.cmd_msg.data)
        self.cmd_msg.data = self.motor_spdw 
        self.pub.publish(self.cmd_msg.data)
        # print(val)


def main():
    s = Sim_kbd(motor_spdw=2400.0)
    with s:
        try:
            while not rospy.is_shutdown():
                s.ctrl_keyb()
                s.rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    main()