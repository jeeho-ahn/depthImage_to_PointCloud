#!/bin/bash

gnome-terminal -e "bash -c 'roslaunch launch/run_rviz.launch;$SHELL'"
sleep 2
gnome-terminal -e "bash -c 'rosbag play bag/rs_camera_sample.bag;$SHELL'"
gnome-terminal -e "bash -c 'rosrun depthImage_to_PointCloud depthImage_to_PointCloud;$SHELL'"
