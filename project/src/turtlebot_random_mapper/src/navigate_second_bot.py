import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class TurtlebotNavigation:
    def __init__(self):
        rospy.init_node('turtlebot_navigation', anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_turtlebot_1 = None
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.move_base_client = SimpleActionClient('/turtlebot_2/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.loginfo("Turtlebot Navigation initialized")

    def odom_callback(self, msg):
        self.pose_turtlebot_1 = msg.pose.pose

    def get_transform(self, source_frame, target_frame):
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
        return transform


    def apply_transform(self, pose, transform):
        pose_in_target = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return pose_in_target.pose

    def navigate_to_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "merged_map" 
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        rospy.loginfo(f"Sending goal to Turtlebot 2: {pose}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        if self.move_base_client.get_state() == 3:
            rospy.loginfo("Turtlebot 2 reached the goal successfully!")
        else:
            rospy.logwarn("Turtlebot 2 failed to reach the goal.")

    def run(self):
        if self.pose_turtlebot_1:
            transform = self.get_transform("map", "merged_map")

            if transform:
                transformed_pose = self.apply_transform(self.pose_turtlebot_1, transform)
                self.navigate_to_pose(transformed_pose)
        else:
            rospy.logwarn("Turtlebot 1's pose not available yet.")


if __name__ == '__main__':
    turtlebot_nav = TurtlebotNavigation()
    turtlebot_nav.run()
