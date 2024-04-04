#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    # Callback function to handle incoming GPS messages
    rospy.loginfo("Received GPS Fix: Latitude={}, Longitude={}, Altitude={}".format(data.latitude, data.longitude, data.altitude))

def gps_subscriber():
    # Initialize ROS node
    rospy.init_node('gps_subscriber_node', anonymous=True)

    # Subscribe to GPS topic
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

    # Spin to keep the node alive
    rospy.spin()

if _name_ == '_main_':
    try:
        gps_subscriber()
    except rospy.ROSInterruptException:
        pass