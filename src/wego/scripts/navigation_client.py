import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

class NavigationClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.goal_list = list()
        
        self.waypoint_1 = MoveBaseGoal()
        self.waypoint_1.target_pose.header.frame_id = "map"
        self.waypoint_1.target_pose.pose.position.x = 18.601542942942007
        self.waypoint_1.target_pose.pose.position.y = -9.865300329485445
        self.waypoint_1.target_pose.pose.orientation.w = 0.9999894849385434
        self.waypoint_1.target_pose.pose.orientation.z = -0.004585849141274627
        
        self.goal_list.append(self.waypoint_1)
        
        self.sequence = 0
        self.start_time = rospy.Time.now()
        
    def run(self):
        if self.client.get_state() != GoalStatus.ACTIVE:
            self.start_time = rospy.Time.now()
            self.sequence = (self.sequence + 1) % 1
            self.client.send_goal(self.goal_list[self.sequence])
            
        else:
            if (rospy.Time.now().to_sec() - self.start_time.to_sec()) > 30.0:
                self.stop()
    
    def stop(self):
        self.client.cancle_all_goals()
        
def main():
    rospy.init_node("navigation_client")
    nc = NavigationClient()
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        nc.run()
        rate.sleep()
    
if __name__ == "__main__":
    main()