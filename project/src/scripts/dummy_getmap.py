#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

class MapReceiver:
    def __init__(self):
        rospy.init_node('map_receiver', anonymous=True)
        self.map_subscriber = rospy.Subscriber('/shared_map', OccupancyGrid, self.map_callback)

/map

    def map_callback(self, data):
        # logic of what we do once getting map
        rospy.loginfo("Received map data.")

if __name__ == '__main__':
    try:
        receiver = MapReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
