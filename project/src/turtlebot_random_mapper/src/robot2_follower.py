#!/usr/bin/env python

# rosrun map_server map_server /home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map.yaml
# roslaunch turtlebot3_navigation amcl.launch map_file:="/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map.yaml"


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class TurtleBotFollower:
    def __init__(self):
        rospy.init_node('turtlebot_follower', anonymous=True)

        self.target_pose = None
        self.pose_subscriber = rospy.Subscriber('/robot1_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()

    def pose_callback(self, data):
        # Save the first TurtleBot's position
        self.target_pose = data.pose.pose
        rospy.loginfo(f"Received target pose: {self.target_pose}")

    def navigate_to_target(self):
        if not self.target_pose:
            rospy.logwarn("No target pose received yet.")
            return

        # Create a goal to move to the target
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.target_pose

        rospy.loginfo("Sending goal to move_base...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached the target!")
        else:
            rospy.logwarn("Failed to reach the target.")

    def run(self):
        rate = rospy.Rate(1)  # Check every second
        while not rospy.is_shutdown():
            if self.target_pose:
                self.navigate_to_target()
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = TurtleBotFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
