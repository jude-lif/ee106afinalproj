#!/usr/bin/env python

import rospy
import os
import subprocess
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import threading
from std_msgs.msg import String


class TurtleBotWallFollower:
    def __init__(self):
        rospy.init_node('turtlebot_wall_follower', anonymous=True)

        # Log and store the starting position
        self.start_position = self.get_robot_position()
        #rospy.set_param("/robot1/start_position", (self.start_position.position.x, self.start_position.position.y))
        rospy.loginfo(f"Start Position: ({self.start_position.position.x}, {self.start_position.position.y})")

        # Save start position to file
        self.position_file_path = os.path.join(os.path.dirname(__file__), "robot_positions.txt")
        with open(self.position_file_path, "w") as file:
            file.write(f"Start Position: ({self.start_position.position.x}, {self.start_position.position.y}, {self.start_position.orientation.x}, {self.start_position.orientation.y}, {self.start_position.orientation.z}, {self.start_position.orientation.w})\n")

        # Publisher and subscriber
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.error_subscriber = rospy.Subscriber('/camera/error', String, self.error_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.wall_distance = 0.25  # Desired distance from the wall in meters
        self.front_detection_distance = 0.24  # Detection range for objects directly in front in meters
        self.emergency_stop_distance = 0.21  # Stop if closer than this to an obstacle
        self.scan_data = None

        # Error
        self.isError = False

        # Map saving directory
        self.map_save_path = "/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/"
        self.map_save_interval = 1  # Save map every 3 seconds

        # Start a thread to save the map periodically
        self.map_saver_thread = threading.Thread(target=self.save_map_periodically)
        self.map_saver_thread.daemon = True
        self.map_saver_thread.start()

    def get_robot_position(self):
        """Helper to get the robot's current position from /odom."""
        odometry_data = rospy.wait_for_message('/odom', Odometry)
        return odometry_data.pose.pose

    def log_final_position(self):
        """Log the robot's final position before shutting down."""
        final_position = self.get_robot_position()
        #rospy.set_param("/robot1/final_position", (final_position.x, final_position.y))
        rospy.loginfo(f"Final Position: ({final_position.position.x}, {final_position.position.y})")

        # Save final position to file
        with open(self.position_file_path, "a") as file:
            file.write(f"Final Position: ({final_position.position.x}, {final_position.position.y}, {final_position.orientation.x}, {final_position.orientation.y}, {final_position.orientation.z}, {final_position.orientation.w})\n")

    def scan_callback(self, data):
        self.scan_data = data

    def error_callback(self, message):
        if message.data == "error":
            self.isError = True
            rospy.logwarn(message.data)

    def save_map_periodically(self):
        while not rospy.is_shutdown():
            if not self.isError:
                rospy.loginfo("Saving map...")
                try:
                    subprocess.call([
                        "rosrun", "map_server", "map_saver",
                        "-f", os.path.join(self.map_save_path, "map"), "map:=/map"
                    ])
                    rospy.loginfo("Map saved successfully.")
                except Exception as e:
                    rospy.logerr(f"Failed to save map: {e}")
                rospy.sleep(self.map_save_interval)

    def follow_wall(self):
        velocity_message = Twist()

        while not rospy.is_shutdown():
            if self.isError:
                return
            if not self.scan_data:
                rospy.logwarn("Waiting for scan data...")
                self.rate.sleep()
                continue

            # Extract distances for specific regions
            front_distances = self.scan_data.ranges[60:120] 
            front = min([dist for dist in front_distances if dist > 0], default=10)

            right_distances = self.scan_data.ranges[0:45] + self.scan_data.ranges[330:]  # Covers 270-300 degrees
            right = min([dist for dist in right_distances if dist > 0], default=10)

            rospy.loginfo(f"Front: {front:.2f}, Right: {right:.2f}")

            # Stop the robot if it is too close to a wall
            if front < self.emergency_stop_distance:
                rospy.logwarn("Emergency stop! Too close to an obstacle.")
                # Reverse for a short duration to create distance
                velocity_message.linear.x = -0.1  # Move backward
                velocity_message.angular.z = 0.1
                self.velocity_publisher.publish(velocity_message)
                rospy.sleep(1)  # Reverse for 1 second
                velocity_message.linear.x = 0.0
            elif front < self.front_detection_distance:
                rospy.loginfo("Obstacle ahead. Stopping and turning left.")
                velocity_message.linear.x = 0.0
                velocity_message.angular.z = 0.6  # Sharper left turn
            elif right > self.wall_distance:
                rospy.loginfo("Too far from wall. Turning right.")
                velocity_message.linear.x = 0.1
                velocity_message.angular.z = -0.3
            elif right < self.wall_distance:
                rospy.loginfo("Too close to wall. Turning left.")
                velocity_message.linear.x = 0.1
                velocity_message.angular.z = 0.3
            else:
                rospy.loginfo("Following wall.")
                velocity_message.linear.x = max(0.05, front - 0.3)  # Slow down as it approaches obstacles
                velocity_message.angular.z = 0.0

            self.velocity_publisher.publish(velocity_message)
            self.rate.sleep()

    def stop_robot(self):
        velocity_message = Twist()
        self.velocity_publisher.publish(velocity_message)
        rospy.sleep(0.5)
        self.log_final_position()  # Log final position when stopping

if __name__ == '__main__':
    try:
        follower = TurtleBotWallFollower()
        rospy.on_shutdown(follower.log_final_position)  # Ensure final position is logged
        follower.follow_wall()
    except rospy.ROSInterruptException:
        pass
