# hw4 for rob499
#
# limiter.py
#
# Cory Hungate
#
# This has been modified from sample code provided for the class by Bill Smart. 
# this is an expansion on the implementation of a publisher from hw2 that 
# includes parameters


# basic imports plus some
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32

class Limiter(Node):
	def __init__(self):
	
		super().__init__('limiter')

		self.pub = self.create_publisher(Float32, 'limited_data', 10)
		
		self.sub = self.create_subscription(Float32, 'oscope', self.callback, 10)

	# This is a callback that will fire every time a message comes in.
	def callback(self, msg):
		
		# Creating a new message that will limit the data at 0.5
		new_msg = Float32()
		if abs(msg.data) > .5:
			new_msg.data = np.sign(msg.data)*.5
		else:
			new_msg.data = msg.data
		
		# I'm going to print the values... its some terminal clutter, but I want a visual key that its working
		self.get_logger().info('Got {0}'.format(new_msg.data))

		#also republishing the newly cropped data
		self.pub.publish(new_msg)


# main function that will be the entry point for this subscriber
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.
	subscriber = Limiter()

	# giving control over to ROS2
	rclpy.spin(subscriber)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	
	main()