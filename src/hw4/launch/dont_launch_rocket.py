# a launch file that starts the action server, and client with cancel enabeled 
# and set to cancel after 3 seconds.
#
# dont_launch_rocket.py
#
# Cory Hungate

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'hw4',
			executable= 'launcher',
			name = 'nasa_server',
		),
		Node(
			package = 'hw4',
			executable= 'canceled_listener',
			name = 'nasa_client',
		),
	])