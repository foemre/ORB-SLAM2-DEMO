#!/usr/bin/env bash
#ORB-SLAM2 euroc demo

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
~/ORB_SLAM2/Examples/Stereo/stereo_euroc ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/Stereo/EuRoC.yaml ~/ORB_SLAM2/MH05/mav0/cam0/data ~/ORB_SLAM2/MH05/mav0/cam1/data ~/ORB_SLAM2/Examples/Monocular/EuRoC_TimeStamps/MH05.txt 