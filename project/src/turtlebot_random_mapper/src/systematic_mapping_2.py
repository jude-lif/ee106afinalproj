#!/usr/bin/env python

import rospy
import os
import subprocess
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
import time

class TurtleBotWallFollower:
    def __init__(self):
        rospy.init_node('turtlebot_wall_follower', anonymous=True)

        # Publisher and subscriber
        self.velocity_publisher = rospy.Publisher('/apple/cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.wall_distance = 0.25  # Desired distance from the wall in meters
        self.front_detection_distance = 0.24  # Detection range for objects directly in front in meters
        self.emergency_stop_distance = 0.21  # Stop if closer than this to an obstacle
        self.scan_data = None

        # Map saving directory
        #self.map_save_path = "/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/"
        #self.map_save_interval = 3  # Save map every 30 seconds

        # Start a thread to save the map periodically
        # self.map_saver_thread = threading.Thread(target=self.save_map_periodically)
        # self.map_saver_thread.daemon = True
        # self.map_saver_thread.start()

    def scan_callback(self, data):
        self.scan_data = data

    # def save_map_periodically(self):
    #     while not rospy.is_shutdown():
    #         rospy.loginfo("Saving map...")
    #         try:
    #             subprocess.call([
    #                 "rosrun", "map_server", "map_saver",
    #                 "-f", os.path.join(self.map_save_path, "map")
    #             ])
    #             rospy.loginfo("Map saved successfully.")
    #         except Exception as e:
    #             rospy.logerr(f"Failed to save map: {e}")
    #         rospy.sleep(self.map_save_interval)

    import time  # Import this for time handling

    def follow_wall(self):
        velocity_message = Twist()
        start_time = rospy.Time.now()  # Record the starting time

        while not rospy.is_shutdown():
            # Stop after 5 seconds
            if rospy.Time.now() - start_time > rospy.Duration(100):  # Check if 5 seconds have passed
                rospy.loginfo("Stopping robot after 5 seconds.")
                self.stop_robot()
                break  # Exit the loop

            if not self.scan_data:
                rospy.logwarn("Waiting for scan data...")
                self.rate.sleep()
                continue

            # Extract distances for specific regions
            front_distances = self.scan_data.ranges[60:120] 
            front = min([dist for dist in front_distances if dist > 0], default=10)

            right_distances = self.scan_data.ranges[0:45] + self.scan_data.ranges[330:]  
            right = min([dist for dist in right_distances if dist > 0], default=10)

            rospy.loginfo(f"Front: {front:.2f}, Right: {right:.2f}")

            # Stop the robot if it is too close to a wall
            if front < self.emergency_stop_distance:
                rospy.logwarn("Emergency stop! Too close to an obstacle.")
                velocity_message.linear.x = -0.1
                velocity_message.angular.z = 0.1
                self.velocity_publisher.publish(velocity_message)
                rospy.sleep(1)
                velocity_message.linear.x = 0.0
            elif front < self.front_detection_distance:
                rospy.loginfo("Obstacle ahead. Stopping and turning left.")
                velocity_message.linear.x = 0.0
                velocity_message.angular.z = 0.6
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
                velocity_message.linear.x = max(0.05, front - 0.3)
                velocity_message.angular.z = 0.0

            self.velocity_publisher.publish(velocity_message)
            self.rate.sleep()


    def stop_robot(self):
        velocity_message = Twist()
        self.velocity_publisher.publish(velocity_message)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        follower = TurtleBotWallFollower()
        follower.follow_wall()
    except rospy.ROSInterruptException:
        pass
