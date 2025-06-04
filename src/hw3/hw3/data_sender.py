#!/usr/bin/env python3

# Example of a service in ROS 2
#
# data_sender.py
#
# Cory Hungate
#
# This is an example of a simple service client in ROS 2 that heavily draws on code 
# provided for the class by Bill Smart.


# standard imports.
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from rob499_msgs.msg import TestPacket
from rob499_msgs.srv import SendData

# In my head, this is primarily a publisher node, hence the name
class BasicPublisher(Node):
	def __init__(self):
		super().__init__('service')

		# Creating the publisher and service
		self.pub = self.create_publisher(TestPacket, 'data', 10)
		self.service = self.create_service(SendData, 'send_data', self.send_data_service)
		
		#now some additional variables I'm gonna need
		self.timer = self.create_timer(2, self.callback)
		self.counter = 0
		self.send_data = True


	def callback(self):
		# Fill in the data in the response type.
		msg = TestPacket()
		msg.send_time = self.get_clock().now().to_msg()
		msg.payload = [1,2,3,4,5]		

		self.pub.publish(msg)

		# Log a message and increment the counter
		# self.get_logger().info(f'Got request # {self.counter} at time {msg.send_time}')
		self.counter += 1


	def send_data_service(self, request, response):
		self.send_data = request.send 

		if self.send_data:
			#if the data is stopped then restarted, we need to restart the timer
			if not self.timer or self.timer.is_canceled:
				self.timer = self.create_timer(2, self.callback)
			response.message = "Publishing started"
			self.get_logger().info("Started publishing data")
		else:
            # Stop the timer, effectively stopping the message publishing
			if self.timer:
				self.timer.cancel()
			response.message = "Publishing stopped"
			self.get_logger().info("Stopped publishing data")
		return response



# This is the entry point for the node.
def main(args=None):

	rclpy.init(args=args)

	publisher = BasicPublisher()

	rclpy.spin(publisher)
	rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == '__main__':
	main()
