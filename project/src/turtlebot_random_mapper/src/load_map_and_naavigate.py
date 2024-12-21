#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
class LoadMapNav:

    def __init__(self):
        rospy.init_node('load_map_and_nav', anonymous=True)

        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.amcl_pose = PoseWithCovarianceStamped()
        self.data_received = False

        # Set initial pose and send goal
        #set_initial_pose(*positions["start"])

        # rospy.spin()    

    def amcl_callback(self, data):
        self.amcl_pose = data
        data_received = True

    def read_positions(self, file_path):
        """Read start and final positions from the text file."""
        with open(file_path, 'r') as file:
            lines = file.readlines()
        positions = {}
        for line in lines:
            if "Start Position" in line:
                start_coords = line.split(":")[1].strip().strip("()").split(", ")
                positions["start"] = (float(start_coords[0]), float(start_coords[1]), float(start_coords[2]), float(start_coords[3]),float(start_coords[4]), float(start_coords[5]))#(-0.24089692533016205, 0.782331705093383)
            elif "Final Position" in line:
                final_coords = line.split(":")[1].strip().strip("()").split(", ")
                positions["final"] = (float(final_coords[0]), float(final_coords[1]), float(final_coords[2]), float(final_coords[3]),float(final_coords[4]), float(final_coords[5]))
        return positions

    def set_initial_pose(self):
        """Publish the robot's initial pose."""
        
        rospy.sleep(1)  # Allow time for the publisher to initialize
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = 0
        initial_pose.pose.pose.position.y = 0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 1.0
        initial_pose.pose.pose.orientation.w = 0.0
        self.pub_initial.publish(initial_pose)
        rospy.loginfo(f"Initial pose set to: x={0}, y={0}")

    def send_goal(self, positions):
        """Publish the goal position."""
        
        rospy.sleep(1)  # Allow time for the publisher to initialize
        goal = PoseStamped()
        goal.header.frame_id = "map"
        transform_x = self.amcl_pose.pose.pose.position.x - positions["start"][0]
        transform_y = self.amcl_pose.pose.pose.position.y - positions["start"][1]
        goal.pose.position.x = -positions["final"][0] #+ transform_x
        goal.pose.position.y = -positions["final"][1] #+ transform_y
        goal.pose.orientation.x = positions["final"][2]
        goal.pose.orientation.y = positions["final"][3]
        goal.pose.orientation.z = positions["final"][4]
        goal.pose.orientation.w = positions["final"][5]
        self.pub.publish(goal)
        rospy.loginfo(f"Goal sent to: x={goal.pose.position.x}, y={goal.pose.position.y}")

if __name__ == "__main__":
    hey = LoadMapNav()

    # Path to the text file
    positions_file = "/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/turtlebot_random_mapper/src/robot_positions.txt"

    # Read positions from the file
    positions = hey.read_positions(positions_file)

    # while not hey.data_received:
    #     rospy.sleep(1)
    print("posted goal")
    hey.send_goal(positions)#*positions["final"])
    hey.set_initial_pose()
