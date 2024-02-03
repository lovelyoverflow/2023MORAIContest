#!/usr/bin/env python3
import cv2
import os
import rospy
import actionlib

from lane_main import LaneFollower
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import Float64

#zfrom obstacle_detector.msg import Obstacles

class NavigationClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.bridge = CvBridge()
        self.left_traffic = False
        self.killed = False
        self.goal_list = list()
        
        self.waypoint_1 = MoveBaseGoal()
        self.waypoint_1.target_pose.header.frame_id = "map"
        self.waypoint_1.target_pose.pose.position.x = 17.53599319448465
        self.waypoint_1.target_pose.pose.position.y = -9.930902082927483
        self.waypoint_1.target_pose.pose.orientation.w = 0.9998693821949046
        self.waypoint_1.target_pose.pose.orientation.z = 0.016162256933356937
        
        self.goal_list.append(self.waypoint_1)

        # 차선주행 관련 sub, pub
        self.car = LaneFollower()
        
        # def _img_cb(msg):
        #     self.car.cv_img = self.car.bridge.compressed_imgmsg_to_cv2(msg)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self._img_cb, queue_size=1)
        
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, callback=self._traffic_cb, queue_size=1)
        
        self.pub_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.pub_steer = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
                
        self.slam = False
        self.sequence = 0
        self.start_time = rospy.Time.now()
        
    
    def _traffic_cb(self, msg):
            if msg.trafficLightStatus == 33:
                self.car.left_traffic = True
            else:
                # print("Status:", msg.trafficLightStatus)
                self.car.left_traffic = False    
                
    def _img_cb(self, msg):
            self.car.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
            
    def run(self):
        if not self.killed:
            os.system("rosnode kill /throttle_interpolator")
            self.killed = True
                
        while self.car.cv_img is None: 
            self.car.rate.sleep()
            
        #인지
        self.car.img_init(self.car.cv_img)
        self.car.img_transform()
        self.car.img_warp(self.car.yellow_range, change_img=False, warp_img_zoomx=self.car.x//2.5)
        self.car.yellow_warped = self.car.warped_img
        self.car.img_warp()
        self.car.adjust_img()#판단
        self.car.sliding_window()

        #판단
        self.car.go_sequence()
        self.car.stop_line()

        #제어
        self.car.control_pub(ctrl=self.car.directControl)
        self.pub_speed.publish(self.car.speed_msg.data)
        self.pub_steer.publish(self.car.steering_msg.data)
        
        cv2.circle(self.car.out_img, (int(self.car.pos), 550), 5, (255, 255, 255))
        cv2.imshow("lane", self.car.out_img)
        cv2.waitKey(1)

    def stop(self):
        self.client.cancle_all_goals()
        
def main():
    rospy.init_node("navigation_client")
    nc = NavigationClient()
    rate = rospy.Rate(30)
    
    nc.client.send_goal(nc.goal_list[nc.sequence])
    while not rospy.is_shutdown():
        if nc.client.get_state() != GoalStatus.SUCCEEDED:
            continue

        nc.run()
        rate.sleep()

if __name__ == "__main__":
    main()