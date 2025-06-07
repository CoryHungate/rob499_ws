# a launch file that opens up RVIZ2 and runs the nodes in this the packages hw7 and hw7_py
#
# bartender_launch.py
#
# Cory Hungate

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

rviz_config_path = os.path.join(
    get_package_share_directory('hw7_py'),
    'config',
    'hw7_config.rviz')

def generate_launch_description():

	print("Loading RViz config from:", rviz_config_path)

	return LaunchDescription([
		Node(
			package = 'hw7',
			executable= 'background_remover',
			name = 'background_remover',
		),
		Node(
			package = 'hw7',
			executable= 'table_remover',
			name = 'table_remover',
		),
		Node(
			package = 'hw7_py',
			executable= 'object_detector',
			name = 'object_detector',
			output='screen',
			parameters=[{'eps': 0.03, 'min_samples': 15}]	
		),
		Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 
                rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen',
			),
	])