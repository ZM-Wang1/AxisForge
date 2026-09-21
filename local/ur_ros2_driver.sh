#!/bin/bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:="ur10e" \
    robot_ip:="192.168.0.100" \
    kinematics_params_file:="/home/arkar/ur10e_calibration.yaml" \
    launch_rviz:=false
