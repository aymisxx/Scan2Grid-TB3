#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class BasicSLAM:
    def __init__(self):
        # Step 1 - Initialization
        rospy.init_node('basic_slam', anonymous=True)

        # Step 2 - Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Map parameters
        self.map_resolution = 0.05  # 5 cm per cell
        self.map_width = 200
        self.map_height = 200
        self.map_origin = [-10.0, -10.0]

        # Step 3 - Publishers
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

        # Initialize pose
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta

        # Initialize map as unknown (-1)
        self.map = -np.ones((self.map_height, self.map_width), dtype=np.int8)

        rospy.loginfo("Basic SLAM node initialized.")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        final_orientation = msg.pose.pose.orientation

        _, _, yaw = tf.transformations.euler_from_quaternion([
            final_orientation.x,
            final_orientation.y,
            final_orientation.z,
            final_orientation.w
        ])

        self.robot_pose = [position.x, position.y, yaw]

    def scan_callback(self, msg):
        angle = msg.angle_min

        for r in msg.ranges:
            # Skip invalid or out-of-range values
            if np.isinf(r) or np.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            # Point in robot frame
            x_r = r * math.cos(angle)
            y_r = r * math.sin(angle)

            # Robot pose in world frame
            x, y, theta = self.robot_pose
            x_w = x + x_r * math.cos(theta) - y_r * math.sin(theta)
            y_w = y + x_r * math.sin(theta) + y_r * math.cos(theta)

            # Convert to map indices
            mx = int((x_w - self.map_origin[0]) / self.map_resolution)
            my = int((y_w - self.map_origin[1]) / self.map_resolution)

            if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                self.map[my][mx] = 100

            angle += msg.angle_increment

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height

        grid_msg.info.origin.position.x = self.map_origin[0]
        grid_msg.info.origin.position.y = self.map_origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = self.map.flatten().tolist()
        self.map_pub.publish(grid_msg)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.publish_map()
            rate.sleep()


if __name__ == '__main__':
    slam = BasicSLAM()
    slam.run()