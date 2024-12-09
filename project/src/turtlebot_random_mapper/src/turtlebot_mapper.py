#!/usr/bin/env python

# catkin_create_pkg turtlebot_random_mapper rospy geometry_msgs sensor_msgs nav_msgs std_msgs
#ssh ubuntu@192.168.1.151
#roslaunch turtlebot3_bringup turtlebot3_robot.launch
#roslaunch turtlebot3_slam turtlebot3_slam.launch
#python turtlebot_mapper.py

#roslaunch turtlebot3_navigation amcl.launch map_file:="/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map.yaml"


# cd $(rospack find turtlebot3_slam)/launch
# vim turtlebot3_slam.launch

# <param name="update_min_d" value="0.2untitled folder 2" />
# <param name="update_min_a" value="0.2" />
# <param name="particles" value="50" />
# roslaunch turtlebot3_navigation amcl.launch map_file:="/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map.yaml"
#  rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal '{ header: { stamp: now, frame_id: "map" }, goal: { target_pose: { header: { stamp: now, frame_id: "map" }, pose: { position: { x: 1.0, y: 1.0, z: 0.0 }, orientation: { w: 1.0 } } } } }'


import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from statistics import median
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped


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
        self.filtered_distances = []

        self.pose_publisher = rospy.Publisher('/robot1_pose', PoseWithCovarianceStamped, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

    def map_callback(self, data):
        self.map_data = data
        rospy.loginfo("Map updated.")
        self.publish_map()

    
    def pose_callback(self, data):
        self.pose_publisher.publish(data)

    def scan_callback(self, data):
        # Front facing
        front_angle_range = range(30, 150)
        num_readings = len(data.ranges)

        # Map angles to indicies
        front_indices = [(i + num_readings) % num_readings for i in front_angle_range]

        # get distances from front angles
        front_distances = [data.ranges[i] for i in front_indices if data.range_min < data.ranges[i] < data.range_max]
        # rospy.loginfo(f"Front distances: {front_distances}")

        # Check if an obstacle is within 0.2 meters
        # obstacle_distance_threshold = 0.2
        # if front_distances and min(front_distances) < obstacle_distance_threshold:
        #     #if any(0.1 < dist < obstacle_distance_threshold for dist in data.ranges):
        #     rospy.logwarn("Obstacle detected directly ahead!")
        #     rospy.logwarn(f"Obstacle detected directly ahead: {min(front_distances):.2f} meters")
        #     self.obstacle_detected = True
        # else:
        #     self.obstacle_detected = False

        if front_distances:
            median_distance = median(front_distances)
            self.filtered_distances = front_distances
            rospy.loginfo_once(f"Median filtered distance: {median_distance:.2f} meters")
        else:
            self.filtered_distances = []

    def error_callback(self, message):
        if message.data == "error":
            self.isError = True
            rospy.logwarn(message.data)

    def publish_map(self):
        if self.map_data:
            # Publish the map to the /shared_map topic
            self.map_publisher.publish(self.map_data)
            rospy.loginfo("Publishing map to /shared_map")

            # Save the map using map_saver
            map_save_path = "/home/cc/ee106a/fa24/class/ee106a-aiv/ros_workspaces/ee106afinalproj/project/src/map" 
            
            rospy.loginfo("Saving the map...")
            subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_save_path], check=True)
            rospy.loginfo(f"Map saved successfully at {map_save_path}.")

    def random_move(self):
        while not rospy.is_shutdown():
            if self.is_obstacle_detected():
                # Stop and rotate randomly to avoid the obstacle
                self.stop_robot()
                rospy.loginfo("Obstacle detected. Avoiding...")
                self.random_rotate()
            else:
                # Randomly decide whether to turn or move forward
                # if random.choice([True, False]):
                #     rospy.loginfo("Random decision: Rotate")
                #     self.random_rotate()
                
                random_distance = random.uniform(0.1, 0.2)
                rospy.loginfo(f"Random decision: Move forward {random_distance:.2f} meters")
                self.move_forward(distance=random_distance)


    def is_obstacle_detected(self):
        obstacle_distance_threshold = 0.3
        return any(dist < obstacle_distance_threshold for dist in self.filtered_distances)

    def move_forward(self, speed=0.1, distance=0.15):
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
        random_angle = random.uniform(30, 90)  # Random angle in degrees
        angular_speed = 0.5  # radians per second
        time_to_rotate = random_angle / (angular_speed * 57.3)  # Convert degrees to radians

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
    except rospy.ROSInterruptException:
        pass
