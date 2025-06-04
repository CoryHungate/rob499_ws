ROB 499 Homework 6
Maintainer: Cory Hungate
License: BSD-3-Clause

-------------------------------------------------------------------------------
Overview and testing:

to test oscope part of the assignment, run the launch files using the following commands:

ros2 launch hw6 limited_oscope.py
	this should launch the oscope, limiter, and plotjuggler. the limiter is set to 0.7 and if you want to see a different limit, the easiest thing to do would be to change it in the launch file

to test the data sender/receiver part of this assignment:
1. test the different services:

	1a. start up the 2 nodes with the commands:
		ros2 run hw6 data_sender
		ros2 run hw6 data_receiver
	
	1b. test the SendData service - note that the service is TRUE by default but no data will log in the terminal. Use the following command to turn the service off. 
		ros2 service call /send_data rob499_msgs/srv/SendData "{send: false}"

	1c. test the EnableLogging service - This service is OFF by default. use the following command to turn the service on. as written, a new csv file called test.csv will be created.
		ros2 service call /enable_logging rob499_msgs/srv/EnableLogging "{enabled: true, filename: test.csv}"

2. test the launch file: the following launch file will open the hw3 and hw6 versions of data_sender and data_receiver as well as an instance of plotjuggler. under topics, select py_latency, py_avg_latency, cpp_latency, and cpp_avg_latency and drag the data into the graph tp visualize. note: these are set to publish at a rate of 2hz

	ros2 launch hw6 comparison_launch.py

-------------------------------------------------------------------------------
This assignment contains 3 different ros2 packages: hw6, hw3, and rob499_msgs. Only the hw6 are discussed below. additional info regarding the hw3 nodes can be found under the hw3 README.

This package contains four ROS2 nodes (hw6 folder) and 2 launch files (launch folder):

oscope.py - publishes a simple sine wave 

limiter.py - node that subscribes to the oscope node and republishes the sine wave with a "limiter" under the parameter "limit". the default value of limit is set to 0.7 both in the node itself and in the launch file. Easiest way to change this value is probably just to change the value in the launch file (limited_oscope.py)

action_client.py - the client that interacts with the server. 

limited_oscope.py - launches the oscope and limiter (with limit set to 0.7) nodes and an instance of plotjuggler

comparison_launch.py - launches the data_sender and data_receiver nodes found in both the hw3 and hw6 packages as well as an instance of plotjuggler. topics have been renamed to reflect the language they were coded in. 

This program additionally contains three custom messages. This are located in the rob499_msgs package included in submission. The services are:

TestPacket.msg
EnableLogging.srv
SendData.srv
        
-------------------------------------------------------------------------------
Collaborators:
For this assigment I made ample use of chat GPT for debugging, the ROS2 documentation for help with services in c++, and stackoverflow for help remembering how to write the ternary operator