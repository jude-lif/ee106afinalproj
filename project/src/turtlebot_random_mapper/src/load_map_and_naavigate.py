#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def read_positions(file_path):
    """Read start and final positions from the text file."""
    with open(file_path, 'r') as file:
        lines = file.readlines()
    positions = {}
    for line in lines:
        if "Start Position" in line:
            start_coords = line.split(":")[1].strip().strip("()").split(", ")
            positions["start"] = (float(start_coords[0]), float(start_coords[1]))
        elif "Final Position" in line:
            final_coords = line.split(":")[1].strip().strip("()").split(", ")
            positions["final"] = (float(final_coords[0]), float(final_coords[1]))
    return positions

def set_initial_pose(x, y):
    """Publish the robot's initial pose."""
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Allow time for the publisher to initialize
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.orientation.w = 1.0
    pub.publish(initial_pose)
    rospy.loginfo(f"Initial pose set to: x={x}, y={y}")

def send_goal(x, y):
    """Publish the goal position."""
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow time for the publisher to initialize
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0
    pub.publish(goal)
    rospy.loginfo(f"Goal sent to: x={x}, y={y}")

if __name__ == "__main__":
    rospy.init_node('turtlebot_navigation', anonymous=True)

    # Path to the text file
    positions_file = "/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/turtlebot_random_mapper/src/robot_positions.txt"

    # Read positions from the file
    positions = read_positions(positions_file)

    # Set initial pose and send goal
    set_initial_pose(*positions["start"])
    send_goal(*positions["final"])

    rospy.spin()
