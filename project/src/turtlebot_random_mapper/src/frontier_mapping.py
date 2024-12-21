#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion

class TurtleBotFrontierExplorer:
    def __init__(self):
        rospy.init_node('turtlebot_frontier_explorer', anonymous=True)

        # Publishers and subscribers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseStamped, self.pose_callback)

        # Map and pose data
        self.map_data = None
        self.robot_pose = None
        self.frontiers = []

        # Service for path planning
        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        self.rate = rospy.Rate(10)

    def map_callback(self, data):
        self.map_data = data
        rospy.loginfo("Map updated.")
        self.identify_frontiers()

    def pose_callback(self, data):
        self.robot_pose = data.pose

    def identify_frontiers(self):
        if not self.map_data:
            return

        # Reshape the map data into a 2D grid
        grid = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        known = (grid >= 0)  # Free or occupied
        unknown = (grid == -1)  # Unexplored

        # Identify frontier cells
        self.frontiers = []
        for y in range(1, grid.shape[0] - 1):
            for x in range(1, grid.shape[1] - 1):
                if unknown[y, x] and np.any(known[y - 1:y + 2, x - 1:x + 2]):
                    self.frontiers.append((x, y))

        rospy.loginfo(f"Identified {len(self.frontiers)} frontiers.")

    def grid_to_world(self, grid_point):
        origin = self.map_data.info.origin.position
        resolution = self.map_data.info.resolution
        wx = origin.x + grid_point[0] * resolution
        wy = origin.y + grid_point[1] * resolution
        return wx, wy

    def world_to_grid(self, wx, wy):
        origin = self.map_data.info.origin.position
        resolution = self.map_data.info.resolution
        gx = int((wx - origin.x) / resolution)
        gy = int((wy - origin.y) / resolution)
        return gx, gy

    def move_to_goal(self, goal):
        if not self.robot_pose:
            rospy.logwarn("Robot pose not available.")
            return False

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.orientation.w = 1.0

        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.pose = self.robot_pose

        try:
            plan = self.make_plan_service(start_pose, goal_pose, 0.1)
            if len(plan.plan.poses) > 0:
                rospy.loginfo(f"Path to goal found: {goal}")
                self.follow_plan(plan.plan.poses)
                return True
            else:
                rospy.logwarn(f"No path to goal: {goal}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Path planning service failed: {e}")
            return False

    def follow_plan(self, poses):
        for pose in poses:
            goal = pose.pose.position
            self.navigate_to_position(goal.x, goal.y)

    def navigate_to_position(self, x, y):
        velocity_message = Twist()
        distance_tolerance = 0.2

        while not rospy.is_shutdown():
            if not self.robot_pose:
                rospy.logwarn("Robot pose not available.")
                return

            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            distance = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)

            if distance <= distance_tolerance:
                rospy.loginfo("Reached goal.")
                break

            # Proportional control for navigation
            velocity_message.linear.x = min(0.5, 0.1 * distance)
            angle_to_goal = math.atan2(y - robot_y, x - robot_x)
            _, _, yaw = euler_from_quaternion([
                self.robot_pose.orientation.x,
                self.robot_pose.orientation.y,
                self.robot_pose.orientation.z,
                self.robot_pose.orientation.w
            ])
            velocity_message.angular.z = 0.5 * (angle_to_goal - yaw)

            self.velocity_publisher.publish(velocity_message)
            self.rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        self.velocity_publisher.publish(Twist())
        rospy.sleep(0.5)

    def start_exploration(self):
        rospy.loginfo("Starting exploration...")
        while not rospy.is_shutdown():
            if not self.frontiers:
                rospy.loginfo("No frontiers left to explore.")
                break

            # Sort frontiers by distance
            self.frontiers.sort(key=lambda f: math.hypot(
                f[0] - self.robot_pose.position.x,
                f[1] - self.robot_pose.position.y
            ))

            for frontier in self.frontiers:
                goal_world = self.grid_to_world(frontier)
                if self.move_to_goal(goal_world):
                    break


if __name__ == '__main__':
    try:
        explorer = TurtleBotFrontierExplorer()
        explorer.start_exploration()
    except rospy.ROSInterruptException:
        pass
