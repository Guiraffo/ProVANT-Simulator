#!/bin/bash
cd ~

export PROVANT_ROS=${HOME}/catkin_ws/src
export TILT_STRATEGIES=${HOME}/catkin_ws/devel/lib/
export TILT_PROJECT=${PROVANT_ROS}/provant_simulator
export TILT_CONFIG=${TILT_PROJECT}/source/Database/simulation_elements/models/real/vant_2comcargaMarcelo/config/config.xml
export TILT_MATLAB=${TILT_PROJECT}/source/Structure/Matlab/
export PROVANT_DATABASE=${TILT_PROJECT}/source/Database
export GAZEBO_MODEL_PATH=${PROVANT_DATABASE}/simulation_elements/models/real
export DIR_ROS=${HOME}/catkin_ws

cd catkin_ws
catkin_make --pkg simulator_msgs
catkin_make

