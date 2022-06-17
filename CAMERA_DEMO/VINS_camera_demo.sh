#!/usr/bin/env bash
#VINS-Mono camera demo

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

roslaunch jetson_csi_cam jetson_csi_cam.launch width:=564 height:=360 fps:=15 cam_name:=camera & sleep 5;
roslaunch vins_estimator euroc_onboard_ogam.launch & sleep 5;
roslaunch vins_estimator vins_rviz.launch & sleep 5;
roslaunch benchmark_publisher publish.launch  sequence_name:=MH_05_difficult & sleep 5; 
rosbag play MH_05_difficult.bag
wait
killall roslaunch
killall gscam 
killall rosbag