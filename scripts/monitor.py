#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import psutil
from openpyxl import Workbook
import os

def monitor_resources():
    # Initialize the ROS node
    rospy.init_node('resource_monitor', anonymous=True)

    # Define publishers for CPU and Memory load
    cpu_pub = rospy.Publisher('/cpu_load', Float32, queue_size=10)
    mem_pub = rospy.Publisher('/memory_load', Float32, queue_size=10)

    # Set the node rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Create an Excel workbook and sheet
    wb = Workbook()
    ws = wb.active
    ws.title = "Resource Monitor"
    ws.append(["Timestamp", "CPU Load (%)", "Memory Load (%)"])  # Add headers

    # File path to save the Excel file
    file_path = os.path.expanduser("~/resource_monitor.xlsx")

    while not rospy.is_shutdown():
        # Get the system's CPU and Memory load
        cpu_load = psutil.cpu_percent(interval=1)
        memory_load = psutil.virtual_memory().percent

        # Create Float32 ROS messages for CPU and Memory
        cpu_msg = Float32()
        cpu_msg.data = cpu_load

        mem_msg = Float32()
        mem_msg.data = memory_load

        # Publish the messages
        rospy.loginfo(f"CPU Load: {cpu_load}% | Memory Load: {memory_load}%")
        cpu_pub.publish(cpu_msg)
        mem_pub.publish(mem_msg)

        # Save data to Excel with a timestamp
        timestamp = rospy.get_time()
        ws.append([timestamp, cpu_load, memory_load])

        # Sleep to maintain the rate
        rate.sleep()

    # Save the workbook when the node is shutdown
    wb.save(file_path)
    rospy.loginfo(f"Data saved to {file_path}")

if __name__ == '__main__':
    try:
        monitor_resources()
    except rospy.ROSInterruptException:
        pass
