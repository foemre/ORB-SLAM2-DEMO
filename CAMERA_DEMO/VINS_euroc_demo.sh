#!/usr/bin/env bash
#VINS-Mono camera demo

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

roslaunch vins_estimator euroc.launch & sleep 5;
roslaunch vins_estimator vins_rviz.launch & sleep 5;
roslaunch benchmark_publisher publish.launch  sequence_name:=MH_05_difficult & sleep 5; 
rosbag play ~/catkin_ws/MH_05_difficult.bag
wait
killall roslaunch
killall gscam 
killall rosbag