# a launch file that opens up the data_sender and data_receiver nodes from 
# hw3 and hw6 as well as an instance of plotjuggler
#
# comparison_launch.py
#
# Cory Hungate

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'hw3',
			executable= 'data_sender',
			name = 'py_data_sender',
			remappings = [('/data', '/py_data')],
		),
		Node(
			package = 'hw3',
			executable= 'data_receiver',
			name = 'py_data_receiver',
			remappings = [('/latency', '/py_latency'), 
				('/raw_latency', '/py_avg_latency'),
				('/data', '/py_data')],
		),
		Node(
			package = 'hw6',
			executable= 'data_sender',
			name = 'cpp_data_sender',
			remappings = [('/data', '/cpp_data')],
		),
		Node(
			package = 'hw6',
			executable= 'data_receiver',
			name = 'cpp_data_receiver',
			remappings = [('/raw_latency', '/cpp_latency'), 
				('/average_latency', '/cpp_avg_latency'),
				('/data', '/cpp_data')],
		),
		 ExecuteProcess(
            cmd=['plotjuggler'],
            name='PlotJuggler',
            output='screen',
            shell=True 
        ),
	])