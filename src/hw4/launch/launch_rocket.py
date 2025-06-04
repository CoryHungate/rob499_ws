# a launch file that starts the action server and client and 
# runs the countdown through to completion
#
# launch_rocket.py
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
			executable= 'listener',
			name = 'nasa_client',
		),
	])