#!/usr/bin/env python3

# Example of publishing and services in ROS 2
#
# data_receiver.py
#
# Cory Hungate
#
# This is an example of a simple publisher with added services in ROS 2.


# Standard imports and some extras
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import csv

from rob499_msgs.msg import TestPacket
from rob499_msgs.srv import EnableLogging
from std_msgs.msg import Float64
import numpy as np


# I'm thinking of this as just a subscriber plus some stuff, hence the name
class BasicSubscriber(Node):
	def __init__(self):
	
		super().__init__('subscriber')

		# Creating the necessary publishers, subscriver, and the service
		self.pub_latency = self.create_publisher(Float64, 'latency', 10)
		self.pub_average = self.create_publisher(Float64, 'raw_latency', 10)

		self.sub = self.create_subscription(TestPacket, 'data', self.callback, 10)

		self.service = self.create_service(EnableLogging, 'enable_logging', self.logging_callback)

		#extra needed variables
		self.enable_logging = False
		self.file_name = ''
		self.arr = np.zeros(10)		#the array for the running average latency
		self.counter = 0


	def callback(self, msg):
		
		#first, I'm creating variables and turning the send_time into a usable form
		current_time = self.get_clock().now()
		current_time_data = current_time.nanoseconds / 1e9
		send_time = Time(
    		seconds=msg.send_time.sec,
    		nanoseconds=msg.send_time.nanosec,
    		clock_type=self.get_clock().clock_type
		)
		difference = (current_time-send_time).nanoseconds / 1e9
		
		latency = Float64()
		average = Float64()
		latency.data = float(difference)
		
		#index to overwrite old latency data 
		index = self.counter % 10
		self.arr[index] = latency.data
		
		#I want to make sure that the average is only on data 
		# that exists, not the zeros from the array intiialization
		if self.counter <10:
			average.data = np.sum(self.arr) / (self.counter + 1)
		else:
			average.data = np.average(self.arr)

		#publishing what I'm supposed to publish
		self.pub_latency.publish(latency)
		self.pub_average.publish(average)

		#logged data will ONLY ever append to the file
		if self.enable_logging and self.file_name:
			with open(self.file_name, mode = 'a', newline = '') as file:
					writer = csv.writer(file)
					writer.writerow([current_time_data, latency.data, average.data])

		#I like to see the published data in the terminal, so I'm including this line
		# self.get_logger().info(f'message took {difference:.6f} seconds to arrive. Running avg is {average.data:.6f}')

		self.counter += 1


	def logging_callback (self, request, response):
		self.enable_logging = request.enabled
		self.file_name = request.filename

		if self.enable_logging:
			self.get_logger().info('logging is enabled')
			response.message = 'logging enabled'
		
		else:
			self.get_logger().info('logging is diabled')
			response.message = 'logging disabled'
		
		self.get_logger().info(f'responding with {response.message}')

		return response

#the entry point for the node
def main(args=None):
	
	rclpy.init(args=args)

	subscriber = BasicSubscriber()

	rclpy.spin(subscriber)

	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()