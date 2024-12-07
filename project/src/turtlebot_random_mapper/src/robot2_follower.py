#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import subprocess

class TurtleBotFollower:
    def __init__(self):
        rospy.init_node('turtlebot_follower', anonymous=True)

        # Subscribers
        rospy.Subscriber('/shared_map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/robot1_pose', PoseWithCovarianceStamped, self.robot1_pose_callback)

        # Publisher for velocity (optional for fine adjustments)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        self.robot1_pose = None
        self.map_received = False

    def map_callback(self, map_data):
        """Callback to handle map data from Robot 1."""
        if not self.map_received:
            # Save the map locally
            map_save_path = "/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/robot2_map"
            rospy.loginfo("Saving map locally...")
            subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_save_path], check=True)
            rospy.loginfo(f"Map saved successfully at {map_save_path}.")

            # Load the map in AMCL for localization
            rospy.loginfo("Loading map in AMCL...")
            subprocess.run(["roslaunch", "turtlebot3_navigation", "amcl.launch", f"map_file:={map_save_path}.yaml"], check=True)
            rospy.loginfo("AMCL localization started.")
            self.map_received = True

    def robot1_pose_callback(self, pose_data):
        """Callback to receive Robot 1's pose."""
        self.robot1_pose = pose_data.pose.pose
        rospy.loginfo_once("Robot 1's pose received.")

    def navigate_to_robot1(self):
        """Navigate to Robot 1's position."""
        if not self.map_received or self.robot1_pose is None:
            rospy.logwarn("Map or Robot 1's pose not received yet. Waiting...")
            rospy.sleep(2)
            return

        rospy.loginfo("Sending goal to move_base...")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.robot1_pose

        self.move_base_client.send_goal(goal)
        success = self.move_base_client.wait_for_result()

        if success:
            rospy.loginfo("Successfully reached Robot 1!")
        else:
            rospy.logwarn("Failed to reach Robot 1.")

    def run(self):
        """Main loop to check data and navigate."""
        rospy.loginfo("Robot 2 is ready to follow Robot 1...")
        while not rospy.is_shutdown():
            if self.map_received and self.robot1_pose:
                self.navigate_to_robot1()
                rospy.sleep(5)  # Pause before retrying navigation
            else:
                rospy.loginfo("Waiting for map and Robot 1's pose...")
                rospy.sleep(2)

if __name__ == '__main__':
    try:
        follower = TurtleBotFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass


