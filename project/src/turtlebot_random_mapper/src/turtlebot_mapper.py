#!/usr/bin/env python

# catkin_create_pkg turtlebot_random_mapper rospy geometry_msgs sensor_msgs nav_msgs std_msgs

#roslaunch turtlebot3_bringup turtlebot3_robot.launch
#roslaunch turtlebot3_slam turtlebot3_slam.launch
#python turtlebot_random_mapper.py

import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class TurtleBotRandomMapper:
    def __init__(self):
        rospy.init_node('turtlebot_mapper', anonymous=True)

        # Publishers and subscribers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_publisher = rospy.Publisher('/shared_map', OccupancyGrid, queue_size=10)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.error_subscriber = rospy.Subscriber('/camera/error', String, self.error_callback)
        self.isError = False

        self.rate = rospy.Rate(10)  # 10 Hz
        self.map_data = None
        self.obstacle_detected = False

    def map_callback(self, data):
        self.map_data = data
        rospy.loginfo("Map updated.")
        self.publish_map()

    def scan_callback(self, data):
        front_angles = range(-15, 16) 
        num_scans = len(data.ranges)
        front_indices = [i % num_scans for i in front_angles]

        # Extract distances for the front angles
        front_distances = [data.ranges[i] for i in front_indices if 0.1 < data.ranges[i] < 3.0]  # Filter valid values

        # Check if there's an obstacle directly ahead within 0.5 meters
        if front_distances and min(front_distances) < 0.05:
            rospy.logwarn("Obstacle detected directly ahead!")
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
    
    def error_callback(self, message):
        if message.data == "error":
            self.isError = True
        print(message.data)

    def publish_map(self):
        if self.map_data:
            rospy.loginfo("Publishing map to /shared_map")
            self.map_publisher.publish(self.map_data)

    def random_move(self):
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Stop and rotate randomly to avoid the obstacle
                self.stop_robot()
                rospy.loginfo("Obstacle detected. Avoiding...")
                self.random_rotate()
            else:
                # Move forward a random distance
                random_distance = random.uniform(0.5, 2.0)
                rospy.loginfo(f"Moving forward: {random_distance:.2f} meters")
                self.move_forward(distance=random_distance)

    def move_forward(self, speed=0.2, distance=1.0):
        velocity_message = Twist()
        velocity_message.linear.x = speed

        start_time = rospy.Time.now().to_sec()
        traveled_distance = 0

        while traveled_distance < distance and not self.obstacle_detected:
            self.velocity_publisher.publish(velocity_message)
            self.rate.sleep()
            current_time = rospy.Time.now().to_sec()
            traveled_distance = speed * (current_time - start_time)

        # Stop the robot
        self.stop_robot()

    def random_rotate(self):
        random_angle = random.uniform(30, 180)
        angular_speed = 0.5 
        time_to_rotate = random_angle / (angular_speed * 57.3) 

        velocity_message = Twist()
        velocity_message.angular.z = angular_speed

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < time_to_rotate:
            self.velocity_publisher.publish(velocity_message)
            self.rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        velocity_message = Twist()
        self.velocity_publisher.publish(velocity_message)
        rospy.sleep(0.5) 

    def start_mapping(self):
        rospy.loginfo("Starting random mapping...")
        self.random_move()

if __name__ == '__main__':
    try:
        mapper = TurtleBotRandomMapper()
        mapper.start_mapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



