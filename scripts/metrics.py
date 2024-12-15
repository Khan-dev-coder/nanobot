#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time

class GlobalPathEvaluator:
    def __init__(self):
        rospy.init_node('global_path_evaluator', anonymous=True)

        # Subscribers
        self.path_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback)
        self.pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        # Publishers
        self.metrics_pub = rospy.Publisher('/global_path_evaluation_metrics', String, queue_size=10)

        # Internal variables
        self.start_time = None
        self.global_path = []  # List of (x, y) tuples
        self.current_pose = None
        self.iterations = 0
        self.reached_goal = False

        rospy.on_shutdown(self.on_shutdown)

    def path_callback(self, msg):
        # Extract the path as a list of (x, y) coordinates
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.start_time = time.time()  # Start timing when the path is received
        rospy.loginfo("Received global path with %d waypoints." % len(self.global_path))

    def pose_callback(self, msg):
        if not self.global_path:
            return  # Wait for path to be published

        self.current_pose = (msg.pose.position.x, msg.pose.position.y)
        self.iterations += 1

        # Check if goal is reached
        goal_x, goal_y = self.global_path[-1]  # Last point in the path
        if self.distance(self.current_pose, (goal_x, goal_y)) < 0.1:  # Goal tolerance
            if not self.reached_goal:
                self.reached_goal = True
                time_taken = time.time() - self.start_time
                self.evaluate_path(time_taken)

    def evaluate_path(self, time_taken):
        path_length = self.calculate_path_length()
        path_smoothness = self.calculate_path_smoothness()
        convergence_rate = self.iterations / len(self.global_path) if len(self.global_path) > 0 else 0

        # Log and publish the metrics
        rospy.loginfo("Time Taken: %.2f seconds" % time_taken)
        rospy.loginfo("Path Length: %.2f meters" % path_length)
        rospy.loginfo("Path Smoothness: %.2f" % path_smoothness)
        rospy.loginfo("Convergence Rate: %.2f" % convergence_rate)
        rospy.loginfo("Number of Iterations: %d" % self.iterations)

        metrics = {
            'time_taken': time_taken,
            'path_length': path_length,
            'path_smoothness': path_smoothness,
            'convergence_rate': convergence_rate,
            'iterations': self.iterations,
        }

        self.metrics_pub.publish(str(metrics))

    def calculate_path_length(self):
        # Compute the total Euclidean distance along the path
        length = 0.0
        for i in range(1, len(self.global_path)):
            length += self.distance(self.global_path[i - 1], self.global_path[i])
        return length

    def calculate_path_smoothness(self):
        # Compute the total angular change between consecutive segments
        smoothness = 0.0
        for i in range(1, len(self.global_path) - 1):
            v1 = np.array(self.global_path[i]) - np.array(self.global_path[i - 1])
            v2 = np.array(self.global_path[i + 1]) - np.array(self.global_path[i])
            angle = self.angle_between(v1, v2)
            smoothness += abs(angle)
        return smoothness

    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    @staticmethod
    def angle_between(v1, v2):
        # Compute the angle between two vectors
        dot_prod = np.dot(v1, v2)
        mag_v1 = np.linalg.norm(v1)
        mag_v2 = np.linalg.norm(v2)
        return math.acos(dot_prod / (mag_v1 * mag_v2 + 1e-8))

    def on_shutdown(self):
        if self.global_path and self.start_time:
            time_taken = time.time() - self.start_time if not self.reached_goal else 0
            path_length = self.calculate_path_length()
            path_smoothness = self.calculate_path_smoothness()
            convergence_rate = self.iterations / len(self.global_path) if len(self.global_path) > 0 else 0

            rospy.loginfo("\nFinal Metrics before Shutdown:")
            rospy.loginfo("Time Taken: %.2f seconds" % time_taken)
            rospy.loginfo("Path Length: %.2f meters" % path_length)
            rospy.loginfo("Path Smoothness: %.2f" % path_smoothness)
            rospy.loginfo("Convergence Rate: %.2f" % convergence_rate)
            rospy.loginfo("Number of Iterations: %d" % self.iterations)

if __name__ == '__main__':
    try:
        evaluator = GlobalPathEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

