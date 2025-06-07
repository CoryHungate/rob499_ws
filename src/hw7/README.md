ROB 499 Homework 7
Maintainer: Cory Hungate
License: BSD-3-Clause

-------------------------------------------------------------------------------
Overview and testing:

IMPORTANT! This package makes use of DBSCAN, and so you must have it installed in order for the person_detector node to work. It is recommended that you create a virtual environment and run the program to avoid issues. 
instructions can be found at https://scikit-learn.org/stable/install.html
NOTE THE ADDITIONAL UBUNTU INSTALL COMMANDS - these are located about halfway down the page

This submission contains 2 packages: hw7 which is a c++ package, and hw7_py which is a python package. There are a total of 3 nodes and 1 launch file contained in these packages. node descriptions are found after this section.

For testing, I recommend having 2 terminal windows open: 1 for the launch file, another for running the bag

1. launch the launch file:
	ros2 launch hw7_py bartender_launch.py

2. play the bag file
	ros2 bag play <bag_file_path>


-------------------------------------------------------------------------------
node_descriptions

This package contains 3 ROS2 nodes (hw7 and hw7_py folders) and 1 launch files (hw7_py/launch):

background_remover.cpp - this node uses the pointcloud library to find a plane and create a bounding box around it in order to remove the background clutter. this node publishes to the topic '/sans_background'

table_remover.cpp - this node also uses the pointcloud library to find a plane, but this node removes that plane. this node publishes to the topics: 
    a) '/sans_table' - the pointcloud2 data with the table removed
    b) '/table_marker' - a marker representing the plane of the table

object_detector.py - this node uses DBSCAN to look at the pointcloud2 published under '/sans_table' to identify objects. This node publishes to 3 topics: 
	a) '/tabletop_objects' - the found-object count published continuously
	b) '/objects' - the centroid of the found-objects
	c) '/data' - the found-object count that is ONLY published when the count changes
	        I used this for debugging but was too lazy to remove it...
	d) '/object_count_marker' - the marker that displays the table object count

bartender_launch.py - launches an instance of RVIZ2 with the config file found at hw7_py/config along with an instance of all the nodes listed above
        
-------------------------------------------------------------------------------
Collaborators:
For this assigment I made ample use of chat GPT for debugging, several stack overflow pages, and the example code provided in class
