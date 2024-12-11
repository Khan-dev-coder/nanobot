#!/usr/bin/env python

import rospy
import psutil
import time
from std_msgs.msg import String  # Replace with the actual message type if different

class PerformanceMonitor:
    def __init__(self):
        rospy.init_node('performance_monitor', anonymous=True)
        
        # Subscriber to the SLAM topic
        self.slam_sub = rospy.Subscriber('/slam_data', String, self.slam_callback)  # Replace String with the actual message type

        # Track last time metrics were logged
        self.last_log_time = time.time()
        self.rate = rospy.Rate(10)  # 10 Hz

    def slam_callback(self, msg):
        # This callback is called whenever a new message arrives on the SLAM topic
        rospy.loginfo("Received SLAM data")

    def log_performance_metrics(self):
        # Calculate elapsed time since last log
        current_time = time.time()
        elapsed_time = current_time - self.last_log_time

        if elapsed_time >= 1.0:
            # Get CPU and memory usage
            cpu_usage = psutil.cpu_percent(interval=None)
            memory_info = psutil.virtual_memory()

            # Log the performance metrics
            rospy.loginfo(f"Time since last log: {elapsed_time:.2f} seconds")
            rospy.loginfo(f"CPU Usage: {cpu_usage:.2f}%")
            rospy.loginfo(f"Memory Usage: {memory_info.percent:.2f}%")

            # Update the last log time
            self.last_log_time = current_time

    def run(self):
        rospy.loginfo("Performance Monitor Node Started")
        while not rospy.is_shutdown():
            self.log_performance_metrics()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        monitor = PerformanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
