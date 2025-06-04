#!/usr/bin/env python3

# laser_scan_filter.py
#
# Cory Hungate
#
# node that subscribes to laser range data and returns data with background noise filtered out.

import rclpy
from rclpy.node import Node

# some speciality imports
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
import numpy as np


class LaserScanFilter(Node):
	def __init__(self):
		super().__init__('laser_scan_filter')

		self.declare_parameter('tolerance', 1.0)
		
		self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
		self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
		self.srv = self.create_service(Trigger, '/snapshot', self.get_snapshot)

		self.tolerance = self.get_parameter('tolerance').value
		self.background_scan = None
		self.most_recent_scan = None
		self.snapshot_taken = False

		self.get_logger().info('Laser_scan_filter has started')
	
	
	def get_snapshot(self, request, response):
		if self.most_recent_scan is not None:
			self.background_scan = list(self.most_recent_scan.ranges)
			self.get_logger().info("Background snapshot manually taken.")
			response.success = True
			response.message = "Snapshot taken"

		else:
			self.get_logger().info("snapshot not taken, no scan yet available")
			response.success = False
			response.message = "Snapshot not taken"
		return response
	

	#runs the laserscan through a filter to remove anything from the background
	#background defined as anything within self.tolerance
	#remember, ranges are in meters
	def scan_callback(self, msg: LaserScan):
		
		self.most_recent_scan = msg

		if self.background_scan is None:
			self.pub.publish(msg)
			return
		
		filtered_ranges = list(msg.ranges)
		for i in range(len(filtered_ranges)):
			current = msg.ranges[i]
			background = self.background_scan[i]

			if np.isfinite(current) and np.isfinite(background):
				if abs(current - background) < self.tolerance:
					filtered_ranges[i] = float('inf')
			
		filtered_msg = LaserScan()
		filtered_msg.header = msg.header
		filtered_msg.angle_min = msg.angle_min
		filtered_msg.angle_max = msg.angle_max
		filtered_msg.angle_increment = msg.angle_increment
		filtered_msg.time_increment = msg.time_increment
		filtered_msg.scan_time = msg.scan_time
		filtered_msg.range_min = msg.range_min
		filtered_msg.range_max = msg.range_max
		filtered_msg.ranges = filtered_ranges
		filtered_msg.intensities = msg.intensities

		self.pub.publish(filtered_msg)
		


def main(args=None):
	
	rclpy.init(args=args)

	publisher = LaserScanFilter()

	rclpy.spin(publisher)

	rclpy.shutdown()


if __name__ == '__main__':

	main()
