ROB 499 Homework 3
Maintainer: Cory Hungate
License: BSD-3-Clause
-------------------------------------------------------------------------------
This program contains two ROS2 nodes:

data_sender.py publishes the time of transmission and a simple array
data_receiver.py subscribes to data_sender and calculates the time it takes to receive the message (called "latency") as well as a running average of the 10 most recent latency calculations

This program additionally contains three custom messages. These are located in the rob499_msgs package included in submission. The services are:

TestPacket.msg - custom message for sending time data
EnableLogging.src - custom service that can turn on or off logging latency and average data to a csv file (default off)
SendData - can stop and restart data_sender (default on)

-------------------------------------------------------------------------------
This is intended to be used with the following commands:

ros2 run hw3 data_sender
        This runs the data_sender publisher
ros2 run hw3 data_receiver
        This runs the data_receiver subscriber

ros2 service call /send_data rob499_msgs/srv/SendData "{send: false}"
        This stops data transmission by data_sender.
ros2 service call /send_data rob499_msgs/srv/SendData "{send: true}"
        This starts data transmission by data_sender.

ros2 service call /enable_logging rob499_msgs/srv/EnableLogging "{enabled: true, filename: <fileName.csv>}"
        This begins logging to filename of your choice 
ros2 service call /enable_logging rob499_msgs/srv/EnableLogging "{enabled: false, filename: ''}"
        This stops data logging
        
-------------------------------------------------------------------------------
Collaborators:
For this assigment I had assistance from Nathan Martin (also in class), stack overflow, the provided example code for the class, and had lots of help from chat GPT for debugging