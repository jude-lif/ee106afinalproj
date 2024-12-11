#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import math

class TurtleBotSearch:
    def __init__(self):
        rospy.init_node('turtlebot_search', anonymous=True)

        # Retrieve start and final positions from TurtleBot1
        self.start_position = rospy.get_param("/robot1/start_position", default=(0.0, 0.0))
        self.final_position = rospy.get_param("/robot1/final_position", default=(0.0, 0.0))

        rospy.loginfo(f"Start Position: {self.start_position}")
        rospy.loginfo(f"Final Position: {self.final_position}")

        # Publisher and subscriber
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_position = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

    def distance_to_target(self, target_position):
        """Calculate Euclidean distance to a target position."""
        if not self.current_position:
            return float('inf')
        return math.sqrt((self.current_position.x - target_position[0]) ** 2 +
                         (self.current_position.y - target_position[1]) ** 2)

    def move_to_position(self, target_position):
        """Move to a specific position."""
        twist = Twist()
        while not rospy.is_shutdown():
            distance = self.distance_to_target(target_position)
            rospy.loginfo(f"Distance to target: {distance:.2f} meters")

            if distance < 0.2:
                rospy.loginfo("Target reached.")
                break

            twist.linear.x = 0.2
            self.velocity_publisher.publish(twist)
            self.rate.sleep()

        # Stop the robot
        twist.linear.x = 0.0
        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        searcher = TurtleBotSearch()
        searcher.move_to_position(searcher.start_position)
        searcher.move_to_position(searcher.final_position)
    except rospy.ROSInterruptException:
        pass
