#!/usr/bin/env python3

# hw4: Example of an action client in ROS 2
#
# action_client.py
#
# Cory Hungate
#
# This is an example of a action client in ROS 2.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rob499_msgs.action import LaunchRocket

class NasaClient(Node):
	def __init__(self, with_cancel=False):
		super().__init__('nasa_client')

		self.client = ActionClient(self, LaunchRocket, 'launch_rocket')

		self.with_cancel = with_cancel


	# This function is a wrapper that will allow us to more conveniently invoke the action.
	def send_goal(self, n):
		goal = LaunchRocket.Goal()
		goal.countdown = n

		self.client.wait_for_server() 
		self.result = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
		self.result.add_done_callback(self.response)


	# Process feedback as it comes in.
	def feedback_cb(self, feedback_msg):
		self.get_logger().info(f'Got feedback: {feedback_msg.feedback.progress}')


	#callback for the cancellation
	def cancel_cb(self, future):
		response = future.result()

		if len(response.goals_canceling) > 0:
			self.get_logger().info('Launch aborted.')
		else:
			self.get_logger().info('Launch failed to abort.')			


	# This callback fires when the action is accepted or rejected.
	def response(self, future):

		self.goal_handle = future.result()

		if not self.goal_handle.accepted:
			self.get_logger().info('countdown rejected')
			return
		
		self.result_handle = self.goal_handle.get_result_async()
		self.result_handle.add_done_callback(self.process_result)

		if self.with_cancel:
			self.timer = self.create_timer(3.0, self.cancel_launch)
	
	def cancel_launch(self):
		self.get_logger().info('Cancelling the goal...')
		cancel_future = self.goal_handle.cancel_goal_async()
		cancel_future.add_done_callback(self.cancel_cb)

		self.timer.cancel()

	# This callback fires when there are results to be had.  This happens when the action
	# is finished.
	def process_result(self, future):

		result = future.result().result

		self.get_logger().info(f'Result: {result.finished}')


# This is the entry point for a complete launch
def main(args=None):
	rclpy.init(args=args)

	client = NasaClient(with_cancel = False)
	client.send_goal(10)

	rclpy.spin(client)

	rclpy.shutdown()

#this is the entry point for a cancelled launch
def main_with_cancel(args=None):
	rclpy.init(args=args)

	client = NasaClient(with_cancel = True)
	client.send_goal(10)

	rclpy.spin(client)

	rclpy.shutdown()

	
# This is the entry point for running the node directly from the command line.
if __name__ == '__main__':
	main()
