#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def add_noise_to_lidar(data):
    # Convert ranges to a numpy array for easy manipulation
    noisy_ranges = np.array(data.ranges)

    # Define noise parameters
    noise_mean = 0.0
    noise_stddev = 0.02

    # Add Gaussian noise
    noisy_ranges += np.random.normal(noise_mean, noise_stddev, len(noisy_ranges))

    # Clip the noisy ranges to be within valid limits
    noisy_ranges = np.clip(noisy_ranges, data.range_min, data.range_max)

    # Create a copy of the original data message
    noisy_data = data
    noisy_data.ranges = list(noisy_ranges)

    # Publish the noisy data
    pub.publish(noisy_data)

rospy.init_node('lidar_noise_adder')
rospy.Subscriber('/nanobot/laser/scan', LaserScan, add_noise_to_lidar)
pub = rospy.Publisher('/noisy_scan', LaserScan, queue_size=10)

rospy.spin()
