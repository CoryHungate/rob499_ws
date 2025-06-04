# Example of publishing in ROS 2 for ROB499 HW2
#
# oscope.py
#
# Cory Hungate
#
# This has been modified from sample code provided for the class by Bill Smart. 
# I have kept some of the original comments that I find helpful as well as 
# added my own

#the standard ROS imports + math
import rclpy
from rclpy.node import Node
import math

# my values are going to be float32 for this assignment
from std_msgs.msg import Float32

class BasicPublisher(Node):
	def __init__(self, wave_frequency):
		# Initializing the parent class and creating a publisher
		super().__init__('publisher')

		self.pub = self.create_publisher(Float32, 'oscope', 10)

		#creating a variable for sampling frequency and another for wave frequency
		self.sampling_frequency = 100
		self.wave_frequency = 2*math.pi*wave_frequency

		# I'm using the counter as my x values
		self.counter = 0.0

		#i'm using the timer as my x values, and going 
		self.timer = self.create_timer(1/self.sampling_frequency, self.callback)

		

	# This callback will be called every time the timer fires.
	def callback(self):
		# Make an Float32 message, calculate the information, and publish it
		msg = Float32()

		#sine wave = A*sin(omega*t + phi), for the base case of omega = 1 hz = 2pi rads
		msg.data = math.sin(self.wave_frequency * self.counter)
		self.pub.publish(msg)

		# Log that we published something.  In ROS2, loggers are associated with nodes, and
		# the idiom is to use the get_logger() call to get the logger.  This has functions
		# for each of the logging levels.
		self.get_logger().info(f'time = {self.counter} and y = {msg.data}')
		self.counter += (1/self.sampling_frequency)


# Creating a different function for each different sine wave I want to generate
# Each wave is basically just a copy/paste of the first with only the frequency changing
def basic_wave(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class
	publisher = BasicPublisher(wave_frequency=1)

	# handing control over to ROS
	rclpy.spin(publisher)

	# Make sure we shutdown everything cleanly... just in case
	rclpy.shutdown()

#wave where the frequency is doubled (f=2)
def short_wave(args=None):

	rclpy.init(args=args)

	publisher = BasicPublisher(wave_frequency=2)

	rclpy.spin(publisher)

	rclpy.shutdown()

#wave where the frequency is halved (f=.5)
def long_wave(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class
	publisher = BasicPublisher(wave_frequency=.5)

	# handing control over to ROS
	rclpy.spin(publisher)

	# Make sure we shutdown everything cleanly... just in case
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	basic_wave()
