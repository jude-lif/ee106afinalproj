#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from std_msgs.msg import Int32


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.robot_close_pub = rospy.Publisher("robot_detected", Int32, queue_size=10)
        self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=10)

        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message (look this message type up online)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx)*depth/self.fx
        Y = (u - self.cy)*depth/self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def process_images(self):
        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)
        # TODO: Define range for cup color in HSV
        # Run `python hsv_color_thresholder.py` and tune the bounds so you only see your cup
        # update lower_hsv and upper_hsv directly

        lower_hsv = np.array([10,145,122]) # TODO: Define lower HSV values for cup color
        upper_hsv = np.array([21, 255, 255]) # TODO: Define upper HSV values for cup color

        # TODO: Threshold the image to get only cup colors
        # HINT: Lookup cv2.inRange()
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # TODO: Get the coordinates of the cup points on the mask
        # HINT: Lookup np.nonzero()
        y_coords, x_coords = np.nonzero(mask)

        self.robot_close_pub.publish(len(x_coords))
        # print("robot found: ", len(x_coords))

        if len(x_coords) > 600:
            os.system("rosnode kill /move_base")

        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
            #print("No points detected. Is your color filter wrong?")
            return

        # Calculate the center of the detected region by 
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))

        # Fetch the depth value at the center
        depth = self.cv_depth_image[center_y, center_x]

if __name__ == '__main__':
    ObjectDetector()
