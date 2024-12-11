import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from statistics import median
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped


class MergedMapSave:
    def __init__(self):
        rospy.init_node('merged_map_save', anonymous=True)

        # Publishers and subscribers
        self.map_subscriber = rospy.Subscriber('/merged_map', OccupancyGrid, self.map_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.map_data = None

    def map_callback(self, data):
        self.map_data = data
        rospy.loginfo("Map updated.")
        self.publish_map()

    def publish_map(self):
        if self.map_data:
            map_save_path = "/home/cc/ee106a/fa24/class/ee106a-ahi/ros_workspaces/ee106afinalproj/project/src/merged_map" 
            
            rospy.loginfo("Saving the map...")
            subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_save_path], check=True)
            rospy.loginfo(f"Map saved successfully at {map_save_path}.")

if __name__ == '__main__':
    try:
        mapper = MergedMapSave()
        mapper.start_mapping()
    except rospy.ROSInterruptException:
        pass
