#!/bin/bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=192.168.1.13
#export ROS_HOSTNAME=192.168.0.182
#export ROS_HOSTNAME=$(hostname -I | tr -d "[:space:]")
echo "Attempting to bring down CAN interfaces"
./shutdownCAN.sh
echo "Attempting to bring up CAN interfaces"
./setupCAN.sh
. devel/setup.bash
roslaunch tfr_launch robot.launch


