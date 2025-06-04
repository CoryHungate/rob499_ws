ROB 499 Homework 2
Maintainer: Cory Hungate
License: Apache-2.0
-------------------------------------------------------------------------------
This program contains two ROS2 nodes:

oscope.py publishes a sine wave of 1hz, .5 hz, or 2 hz
limiter.py subscribes to oscope.py and limits the values to  between -0.5 & 0.5
-------------------------------------------------------------------------------
This is intended to be used with the following commands:

ros2 run hw2 oscope
        This runs the 1hz wave
ros2 run hw2 fast_wave
        This runs the 2hz wave
ros2 run hw2 slow_wave
        This runs the .5hz wave
ros2 run hw2 limiter
        This runs the limiter
-------------------------------------------------------------------------------
Collaborators:
For this assigment I had assistance from Nathan Martin (also in class), stack overflow, and the provided example code for the class