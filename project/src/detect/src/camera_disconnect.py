#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def simulate_camera_disconnection():
    rospy.loginfo("Camera failure simulation node started.")
    pub = rospy.Publisher('/camera/error', String, queue_size=10)
    r = rospy.Rate(10) 
    rospy.sleep(5)


    while not rospy.is_shutdown():
    
          
        #warning_message = "[139829304096512] (backend-v4l2.cpp:1013) Frames didn't arrive within 5 seconds"
        warning_message = "error"
        pub.publish(warning_message)

    r.sleep()

if __name__ == '__main__':
    rospy.init_node('camera_failure_simulation', anonymous=True)
    try:
        simulate_camera_disconnection()
    except rospy.ROSInterruptException:
        pass