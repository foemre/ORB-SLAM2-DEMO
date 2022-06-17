#!/usr/bin/env bash
#ORB-SLAM2 camera demo

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/nvidia/ORB_SLAM2/Examples/ROS
roslaunch jetson_csi_cam jetson_csi_cam.launch width:=752 height:=480 fps:=20 cam_name:=camera & sleep 5; bg;
rosrun ORB_SLAM2 Mono ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/Monocular/EuRoC.yaml 
wait
killall roslaunch
killall gscam 
killall rosbag