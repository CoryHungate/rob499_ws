# Example of a publisher that takes parameters in ROS 2 for ROB499 HW2
#
# oscope.py
#
# Cory Hungate
#
# This has been modified from sample code provided for the class by Bill Smart. 
# this is an expansion on the implementation of a publisher from hw2 that 
# includes parameters

#the standard ROS imports + math
import rclpy
from rclpy.node import Node
import math

#these are the imports to handle parameters
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

from std_msgs.msg import Float32
from rob499_msgs.srv import SendData

class OscopePublisher(Node):
	def __init__(self):
		# Initializing the parent class and creating a publisher
		super().__init__('publisher')

		self.declare_parameter('frequency', 1.0)
		self.declare_parameter('clamp', 2.0)

		self.pub = self.create_publisher(Float32, 'oscope', 10)
		self.service = self.create_service(SendData, 'send_data', self.send_data_service)

		self.sampling_frequency = 100
		self.counter = 0.0
		self.frequency = self.get_parameter('frequency').value
		self.clamp = self.get_parameter('clamp').value
		self.send_data = True

		self.timer = self.create_timer(1/self.sampling_frequency, self.callback)

		#creating an event handler for registering parameter changes and the callbacks
		self.handler = ParameterEventHandler(self)

		self.frequency_handler = self.handler.add_parameter_callback(
			parameter_name = 'frequency', 
			node_name ='publisher', 
			callback = self.frequency_callback
		)

		self.clamp_handler = self.handler.add_parameter_callback(
			parameter_name = 'clamp', 
			node_name ='publisher', 
			callback = self.clamp_callback
		)	

	def frequency_callback(self, param):
		self.frequency = parameter_value_to_python(param.value)
		self.get_logger().info(f'Frequency changed to {self.frequency}')

	def clamp_callback(self, param):
		self.clamp = parameter_value_to_python(param.value)
		self.get_logger().info(f'clamp changed to {self.clamp}')

	# This callback will be called every time the timer fires.
	def callback(self):
		msg = Float32()

		raw_value = math.sin(2 * math.pi * self.frequency * self.counter)
		msg.data = max(-self.clamp, min(self.clamp, raw_value))
		self.pub.publish(msg)

		#self.get_logger().info(f'time = {self.counter} and y = {msg.data}')
		self.counter += (1/self.sampling_frequency)
	
	def send_data_service(self, request, response):
		self.send_data = request.send

		if self.send_data:
			if not self.timer or self.timer.is_canceled():
				self.timer = self.create_timer(1/self.sampling_frequency, self.callback)
			response.message = 'publishing started'
			self.get_logger().info('data publishing started')
		
		else:
			if self.timer:
				self.timer.cancel()
			response.message = 'publishing stopped'
			self.get_logger().info('data publishing stopped')
		
		return response


def basic_wave(args=None):
	
	rclpy.init(args=args)

	publisher = OscopePublisher()

	rclpy.spin(publisher)

	rclpy.shutdown()


if __name__ == '__main__':
	
	basic_wave()
