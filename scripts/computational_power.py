#!/usr/bin/env python

import rospy
import psutil
import time
import pandas as pd

class PerformanceMonitor:
    def __init__(self):
        rospy.init_node('computational_power', anonymous=True)
        
        # Initialize the start time for monitoring
        self.start_time = time.time()
        
        # Set the rate at which to log performance metrics (e.g., every second)
        self.log_interval = 1.0  # 1 second
        self.rate = rospy.Rate(1)  # 1 Hz
        
        # Initialize lists to store CPU and memory usage data
        self.timestamps = []
        self.cpu_usages = []
        self.memory_usages = []

    def log_performance_metrics(self):
        # Calculate elapsed time
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Get CPU and memory usage
        cpu_usage = psutil.cpu_percent(interval=None)
        memory_info = psutil.virtual_memory()
        
        # Store data in lists
        self.timestamps.append(elapsed_time)
        self.cpu_usages.append(cpu_usage)
        self.memory_usages.append(memory_info.percent)
        
        # Log the performance metrics
        rospy.loginfo(f"Elapsed Time: {elapsed_time:.2f} seconds")
        rospy.loginfo(f"CPU Usage: {cpu_usage:.2f}%")
        rospy.loginfo(f"Memory Usage: {memory_info.percent:.2f}%")

    def save_to_excel(self):
        # Create a DataFrame from the collected data
        data = {
            'Timestamp': self.timestamps,
            'CPU Usage (%)': self.cpu_usages,
            'Memory Usage (%)': self.memory_usages
        }
        df = pd.DataFrame(data)
        
        # Save DataFrame to an Excel file
        df.to_excel('performance_metrics.xlsx', index=False, engine='openpyxl')

    def run(self):
        rospy.loginfo("Performance Monitor Node Started")
        try:
            while not rospy.is_shutdown():
                self.log_performance_metrics()
                self.rate.sleep()  # Sleep to maintain the rate
        finally:
            # Ensure the data is saved when the node shuts down
            self.save_to_excel()
            rospy.loginfo("Performance Monitor Node Shutting Down")

if __name__ == '__main__':
    try:
        monitor = PerformanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Performance Monitor Node Interrupted")
