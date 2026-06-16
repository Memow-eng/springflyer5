#!/bin/bash
sudo chmod 777 /dev/ttyACM0
sleep 2

roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 &
sleep 10

roslaunch realsense2_camera rs_camera.launch infra_width:=640 infra_height:=480 depth_width:=640 depth_height:=480 &
sleep 10


roslaunch vins fast_drone_250.launch
