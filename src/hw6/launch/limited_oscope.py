# a launch file that starts three different oscilloscope nodes 
# and opens an instance of plotjuggler.
#
# limited_launch.py
#
# Cory Hungate

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'hw6',
			executable= 'oscope',
			name = 'oscope',
		),
		Node(
			package = 'hw6',
			executable= 'limiter',
			name = 'limiter',
			parameters = [{'limit': 0.7}],
		),
		 ExecuteProcess(
            cmd=['plotjuggler'],
            name='PlotJuggler',
            output='screen',
            shell=True 
        ),
	])