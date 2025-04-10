#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def add_gaussian_noise(scan_data, mean=0.0, stddev=0.08):
    noisy_ranges = []
    for r in scan_data.ranges:
        if np.isfinite(r):
            noisy_r = r + np.random.normal(mean, stddev)
            noisy_ranges.append(max(scan_data.range_min, min(noisy_r, scan_data.range_max)))
        else:
            noisy_ranges.append(r)  # Keep infinite values unchanged
    return noisy_ranges

def scan_callback(msg):
    noisy_scan = LaserScan()
    noisy_scan.header = msg.header
    noisy_scan.angle_min = msg.angle_min
    noisy_scan.angle_max = msg.angle_max
    noisy_scan.angle_increment = msg.angle_increment
    noisy_scan.time_increment = msg.time_increment
    noisy_scan.scan_time = msg.scan_time
    noisy_scan.range_min = msg.range_min
    noisy_scan.range_max = msg.range_max
    noisy_scan.ranges = add_gaussian_noise(msg, mean=0.0, stddev=0.08)
    noisy_scan.intensities = msg.intensities
    
    pub.publish(noisy_scan)

def main():
    global pub
    rospy.init_node('laser_noise_adder', anonymous=True)
    rospy.Subscriber('/nanobot/laser/scan', LaserScan, scan_callback)
    pub = rospy.Publisher('/scan_noisy', LaserScan, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
