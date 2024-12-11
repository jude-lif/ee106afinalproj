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


class AppleMapPublisher:
    def __init__(self):
        rospy.init_node('apple_map_publisher', anonymous=True)

        # Publishers and subscribers
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.map_data = None

    def map_callback(self, data):
        self.map_data = data
        rospy.loginfo("Map updated.")
        self.publish_map()

    def publish_map(self):
        if self.map_data:
            map_save_path = "/home/cc/ee106a/fa24/class/ee106a-ahi/ros_workspaces/ee106afinalproj/project/src/map2" 
            
            rospy.loginfo("Saving the map...")
            subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_save_path], check=True)
            rospy.loginfo(f"Map saved successfully at {map_save_path}.")

if __name__ == '__main__':
    try:
        mapper = AppleMapPublisher()
        mapper.start_mapping()
    except rospy.ROSInterruptException:
        pass
