#!/usr/bin/env python3
import cv2
import os
import rospy
import math
import actionlib

from lane_main import LaneFollower
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import Int32, String, Float32, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped

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
        self.waypoint_1.target_pose.pose.position.x = 18.314141071314296
        self.waypoint_1.target_pose.pose.position.y = -10.213097824867216
        self.waypoint_1.target_pose.pose.orientation.w = 0.9999999999
        self.waypoint_1.target_pose.pose.orientation.z = 0.0000001
        
        self.goal_list.append(self.waypoint_1)
        
        # 차선주행 관련 sub, pub
        self.car = LaneFollower()
        ################ 그니까 슬램할 때 라이다 콜백 실행안된느데 왜 됨???????????????????????
        # def _img_cb(msg):
        #     self.car.cv_img = self.car.bridge.compressed_imgmsg_to_cv2(msg)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self._img_cb, queue_size=1)
        rospy.Subscriber("/lidar_warning", String, self.lidar_warning_callback)
        rospy.Subscriber("/object_condition", Float32, self.object_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, callback=self._traffic_cb, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=self.amcl_callback, queue_size=1)
        
        rospy.Subscriber("/obstacle_pos", String, self.obstacle_pos_callback)

        self.current_lane_pub = rospy.Publisher("/current_lane", Float64, queue_size=1)  # 1 = right, 2 = left
        self.pub_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.pub_steer = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
                
        self.slam = False
        self.sequence = 0
        self.start_time = rospy.Time.now()
        self.current_position = None
        
    def obstacle_pos_callback(self, msg):
        self.car.obstacle_state = msg.data
    
    def _traffic_cb(self, msg):
            if msg.trafficLightStatus == 33:
                self.car.left_traffic = True
            else:
                # print("Status:", msg.trafficLightStatus)
                self.car.left_traffic = False    
    
    def amcl_callback(self, msg):
        self.current_position = msg
    
    def _img_cb(self, msg):
            self.car.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)

    def lidar_warning_callback(self, msg):
        self.car.lidar_warning = msg.data
        if self.car.lidar_warning == "safe":
            self.car.is_safe = True
        elif self.car.lidar_warning == "WARNING":
            self.car.is_safe = False
            #rospy.loginfo("WARNING !! ")
        else:
            # rospy.loginfo("엥?????????????????????//")
            pass

    def object_callback(self, msg):
        self.car.y_list.append(msg.data)
        # if not self.car.dynamic_flag or len(self.car.y_list) <= 19:
        #     self.car.y_list.append(msg.data)
        #     if len(self.car.y_list) >= 21:
        #         del self.car.y_list[0]
        # else:
        #     rospy.logwarn("Unknown warning state!")
    
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
        self.car.go_sequence(self.current_position)
        self.car.stop_line()

        #제어
        self.car.control_pub(ctrl=self.car.directControl)
        self.current_lane_pub.publish(self.car.lane_msg)
        self.pub_speed.publish(self.car.speed_msg.data)
        self.pub_steer.publish(self.car.steering_msg.data)
        
        # cv2.circle(self.car.out_img, (int(self.car.pos), 550), 5, (255, 255, 255))
        # cv2.imshow("lane", self.car.out_img)
        # cv2.waitKey(1)

    def stop(self):
        self.client.cancle_all_goals()
        
def main():
    rospy.init_node("navigation_client")
    nc = NavigationClient()
    rate = rospy.Rate(30)
    
    nc.client.send_goal(nc.goal_list[nc.sequence])
    nc.client.wait_for_result()
    while not rospy.is_shutdown():
        if nc.client.get_state() != GoalStatus.SUCCEEDED:
            continue
        nc.run()
        rate.sleep()

if __name__ == "__main__":
    main()