
ros2 bag play ~/bag_files/hw5_rosbag_quori/hw5_good_good/

ros2 service call /snapshot std_srvs/srv/Trigger

ROB 499 Homework 5
Maintainer: Cory Hungate
License: BSD-3-Clause

-------------------------------------------------------------------------------
Basic instrucitons

I did not get around to implmenting the camera functionality, so don't bother looking for it

IMPORTANT! This package makes use of DBSCAN, and so you must have it installed in order for the person_detector node to work. It is recommended that you create a virtual environment and run the program to avoid issues. 
instructions can be found at https://scikit-learn.org/stable/install.html
NOTE THE ADDITIONAL UBUNTU INSTALL COMMANDS - these are located about halfway down the page

to test the assignment, enter your shiny new virtual environment and run the launch file:
	ros2 launch hw5 launch.py

play the bag file

call the snapshot service with the command: 
	ros2 service call /snapshot std_srvs/srv/Trigger

You should see smaller red dots (unfiltered data), larger blue dots (filtered data), a yellow circle that is the robots personal space, large white spheres that are the detected persons, and a large white square when the detected persons enter the robots personal space

NOTES:
when I colcon build this package a get a whole mess of alerts. they don't seem to hurt anything so I've been ignoring them

-------------------------------------------------------------------------------
Collaborators:
For this assigment I made ample use of chat GPT for debugging and serious help for how to parse the various data classes

-------------------------------------------------------------------------------
additional things for me to remember
source sklearn-env/bin/activate