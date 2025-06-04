#!/usr/bin/env python3

# hw4: Example of an action server in ROS 2
#
# action_server.py
#
# Cory Hungate
#
# This is an example of a action server in ROS 2.


# Pull in the stuff we need from rclpy.
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rob499_msgs.action import LaunchRocket
from time import sleep


class NasaActionServer(Node):
	def __init__(self):
		super().__init__('nasa_server')

		self.server = ActionServer(self, LaunchRocket, 'launch_rocket', self.callback, 
			callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_callback)

	def callback(self, goal_handle):

		countdown = goal_handle.request.countdown

		feedback_msg = LaunchRocket.Feedback()
		
		self.get_logger().info(f'Got Launch Timer of {countdown} Seconds')

		result = LaunchRocket.Result()

		for i in range(countdown, -1, -1):
			
			#checking for cancellation
			if goal_handle.is_cancel_requested:
				goal_handle.canceled()
				self.get_logger().info('Launch Aborted!')
				result.finished = 'Launch Aborted, there will be no liftoff!'
				return result

			feedback_msg.progress = (f'T-minus {i} seconds until liftoff')
			goal_handle.publish_feedback(feedback_msg)
			self.get_logger().info(feedback_msg.progress)

			sleep(1)

		goal_handle.succeed()
		result.finished = 'We Have Liftoff!'
		self.get_logger().info(result.finished)

		return result

	# This callback fires when a cancellation request comes in.
	def cancel_callback(self, goal_handle):
		self.get_logger().info('Abort Launch Requested')
		return CancelResponse.ACCEPT


# This is the entry point.
def main(args=None):
	rclpy.init(args=args)

	# Set up a node to do the work.
	server = NasaActionServer()

	rclpy.spin(server, MultiThreadedExecutor())

	rclpy.shutdown()


# This is the entry point for running the node directly from the command line.
if __name__ == '__main__':
	main()
