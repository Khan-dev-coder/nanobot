#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 

def callback(msg):
    x = msg.ranges[359]
    print(x)
    if x >= 5.6690:
        print("Okay")
    else:
        print("Ghapla")


rospy.init_node("scan values")
sub = rospy.Subscriber("/nanobot/laser/scan", LaserScan, callback=callback)
rospy.spin()

