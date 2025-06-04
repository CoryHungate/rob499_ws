# a launch file that opens up RVIZ2 and runs several of the nodes in this package
#
# launch.py
#
# Cory Hungate

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

rviz_config_path = os.path.join(
    get_package_share_directory('hw5'),
    'config',
    'hw5_rviz2_config.rviz')

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'hw5',
			executable= 'filtered_laserscan',
			name = 'laser_scan_filter',
			output='screen',
            parameters=[{'tolerance': 1.0}]	
		),
		Node(
			package = 'hw5',
			executable= 'people_detector',
			name = 'people_detector',
			output='screen',
		),
		Node(
			package = 'hw5',
			executable= 'personal_space',
			name = 'personal_space',
			output='screen',
		),
		Node(
			package = 'hw5',
			executable= 'intrusion_detector',
			name = 'intrusion_detector',
			output='screen',
		),
		Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 
                rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
	])