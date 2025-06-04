# a launch file that starts three different oscilloscope nodes 
# and opens an instance of plotjuggler.
#
# launch_oscopes.py
#
# Cory Hungate

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'hw4',
			executable= 'oscope',
			name = 'wave1',
			parameters = [{'frequency': 1.0, 'clamp': 1.5}],
			remappings = [('/oscope', '/wave1')],
		),
		Node(
			package = 'hw4',
			executable= 'oscope',
			name = 'wave2',
			parameters = [{'frequency': 5.0, 'clamp': 1.5}],
			remappings = [('/oscope', '/wave2')],
		),
		Node(
			package = 'hw4',
			executable= 'oscope',
			name = 'wave3',
			parameters = [{'frequency': 10.0, 'clamp': 0.7}],
			remappings = [('/oscope', '/wave3')],
		),

		ExecuteProcess(
			cmd = ['/snap/bin/plotjuggler'],
			output = 'screen',
		),
	])