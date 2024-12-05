#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


isError = False

def callback(message):
	global isError
	if message.data == "error":
		isError = True
	print(message.data)

def new_talker():

	global isError

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber("/camera/error", String, callback)
	r = rospy.Rate(10) # 10hz

	# Loop until the node is killed with Ctrl-C
	while not rospy.is_shutdown():
	    msg = Twist()
	    lin = Vector3()
	    lin.x = 1
	    lin.y = 0
	    lin.z = 0
	    ang = Vector3()
	    ang.x = 0
	    ang.y = 0
	    ang.z = 0
	    if isError:
	    	return
	    msg.linear = lin
	    msg.angular = ang
	    # Publish our string to the 'chatter_talk' topic
	    pub.publish(msg)


	    # Use our rate object to sleep until it is time to publish again
	r.sleep()
if __name__ == '__main__':

	# Run this program as a new node in the ROS computation graph called /talker.
	rospy.init_node('new_talker', anonymous=True)

	# Check if the node has received a signal to shut down. If not, run the
	# talker method.
	try:
	    new_talker()
	except rospy.ROSInterruptException: pass