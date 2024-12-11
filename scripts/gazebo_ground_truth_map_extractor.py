#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
import cv2
import os

class GazeboGroundTruthMap:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gazebo_ground_truth_map_extractor', anonymous=True)

        # Parameters
        self.map_size = (1000, 1000)  # Size of the map in pixels (width, height)
        self.resolution = 0.10  # Resolution in meters per pixel
        self.map_center = (self.map_size[0] // 2, self.map_size[1] // 2)  # Center of the map
        self.occupancy_grid = np.zeros(self.map_size, dtype=np.uint8)  # Create an empty grid

        # Subscribe to /gazebo/model_states to get all model positions
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        # Wait until we receive the model states
        rospy.wait_for_message("/gazebo/model_states", ModelStates)
        rospy.loginfo("Received Gazebo model states. Generating map...")

    def model_states_callback(self, data):
        # Extract model names and their positions
        for i, model_name in enumerate(data.name):
            # Ignore ground plane and robot models if needed
            if model_name in ["ground_plane", "robot"]:
                continue

            # Get model position
            pos = data.pose[i].position
            x, y = pos.x, pos.y

            # Convert (x, y) in meters to grid coordinates
            grid_x = int(self.map_center[0] + x / self.resolution)
            grid_y = int(self.map_center[1] - y / self.resolution)  # Flip y-axis for image coordinate

            # Mark this grid cell as occupied
            if 0 <= grid_x < self.map_size[0] and 0 <= grid_y < self.map_size[1]:
                self.occupancy_grid[grid_y, grid_x] = 255  # Mark occupied cells as 255 (white)

    def save_map(self, map_filename):
        # Save the map as a .pgm image
        map_image = 255 - self.occupancy_grid  # Invert colors for proper occupancy map
        cv2.imwrite(f"{map_filename}.pgm", map_image)

        # Save the .yaml metadata file
        with open(f"{map_filename}.yaml", 'w') as yaml_file:
            yaml_content = f"""image: {os.path.basename(map_filename)}.pgm
resolution: {self.resolution}
origin: [-{self.map_size[0] * self.resolution / 2}, -{self.map_size[1] * self.resolution / 2}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
            yaml_file.write(yaml_content)

        rospy.loginfo(f"Map saved as {map_filename}.pgm and {map_filename}.yaml")

if __name__ == '__main__':
    # Create an instance of the GazeboGroundTruthMap class
    ground_truth_extractor = GazeboGroundTruthMap()

    # Spin to process incoming messages
    rospy.sleep(2.0)  # Wait for map to be generated
    ground_truth_extractor.save_map("ground_truth_map")

    # Keep the node alive
    rospy.loginfo("Ground truth map extraction complete.")
    rospy.spin()
