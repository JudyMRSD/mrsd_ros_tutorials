Introduction

This package will be used to illustrate the basics of ROS and highlight some of its great features.

Dynamic Reconfigure
To get started with the dynamic reconfigure example do execute the following commands. You may have to 
play with the settings in rviz to correctly see the box marker.

Terminal 1:
	rosrun rviz rviz
Terminal 2:
	roslaunch mrsd_ros_tutorials box



Pointclouds in ROS

To see some boilerplate code on PCL and ROS, try running the node
	rosrun mrsd_ros_tutorials pointcloud_transform_node

You will need to play back a rosbag that I will provide to you in part 3 in order to see any output. The node is looking for a pointcloud2 on the topic camera/depth_registered/points_throttle. 
Use RVIZ to visualize.


Finally, There are 3 classes to demostrate different node designs that you will encounter as your projects grow.  We will discuss the pros and cons
of each in the class ROS lecture.  The examples are run with:
	
	rosrun mrsd_ros_tutorials abstracted_class_node
	rosrun mrsd_ros_tutorials in_source_node
	rosrun mrsd_ros_tutorials linear_node