ROB 499 Homework 4
Maintainer: Cory Hungate
License: BSD-3-Clause

-------------------------------------------------------------------------------
TLDR

to test the assignment, run the launch files using the following commands:

ros2 launch hw4 launch_oscopes.py
ros2 launch hw4 launch_rocket.py
ros2 launch hw4 dont_launch_rocket.py

-------------------------------------------------------------------------------
This program contains three ROS2 nodes (hw4 folder) and 3 launch files (launch folder):

oscope.py - publishes a simple sine wave and takes in the parameters "frequency" and "clamp"
action_server.py - is a simple implementation of an action server that performs a countdown starting from 10
action_client.py - the client that interacts with the server. 

launch.py - launches three instances of the oscope.py node and launches plotjuggler. The three waves should appear as Wave1, Wave2, and Wave3 in plotjuggler
launch_rocket.py launches the action server and action client. This will complete the launch sequence 
dont_launch_rocket.py launches the action server and action client. This will cancel the launch sequence partway through the launch

This program additionally contains one custom messages. This are located in the rob499_msgs package included in submission. The services are:

LaunchRocket.action

-------------------------------------------------------------------------------
This is intended to be used with the following commands:

ros2 launch hw4 launch_oscopes.py
    this will launch three different versions of a sine wave with different frequencies and open an instance of plotjuggler

ros2 launch hw4 launch_rocket.py
	this will launch the action server and action client. This will complete the launch sequence 

ros2 launch hw4 dont_launch_rocket.py
	this will launch the action server and action client. This will cancel the launch sequence partway through the launch
        
-------------------------------------------------------------------------------
Collaborators:
For this assigment I made ample use of chat GPT for debugging