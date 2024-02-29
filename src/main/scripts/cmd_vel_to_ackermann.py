#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class CmdToAck():
    def __init__(self):
        rospy.init_node("cmd_vel_to_ackerman", anonymous=True)
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=10)
        self.ack_pub = rospy.Publisher("/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=10)
        self.wheelbase = 0.26
        self.frame_id = "odom"
    
    def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0
        
        radius = v / omega
        return math.atan(wheelbase / radius)
    
    def cmd_callback(self, data):
        v = data.linear.x
        steering = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)
        
        msg=AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = steering
        msg.drive.speed = v
        self.ack_pub.publish(msg)

def main():
    try:
        cmd_to_ack = CmdToAck()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == "__main__":
    main()