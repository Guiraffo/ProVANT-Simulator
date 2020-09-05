#!/bin/bash
#
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
#
# Change directory to the current user home dir
cd ~

# Append environment variables to the current user .bashrc file.
export PROVANT_ROS=${HOME}/catkin_ws/src
echo "export PROVANT_ROS=${HOME}/catkin_ws/src" >> ${HOME}/.bashrc
export TILT_STRATEGIES=${HOME}/catkin_ws/devel/lib/
echo "export TILT_STRATEGIES=${HOME}/catkin_ws/devel/lib/" >> ${HOME}/.bashrc
export TILT_PROJECT=${PROVANT_ROS}/ProVANT-Simulator_Developer
echo "export TILT_PROJECT=${PROVANT_ROS}/ProVANT-Simulator_Developer" >> ${HOME}/.bashrc
export TILT_MATLAB=${TILT_PROJECT}/source/Structure/Matlab/
echo "export TILT_MATLAB=${TILT_PROJECT}/source/Structure/Matlab/" >> ${HOME}/.bashrc
export PROVANT_DATABASE=${TILT_PROJECT}/source/Database
echo "export PROVANT_DATABASE=${TILT_PROJECT}/source/Database" >> ${HOME}/.bashrc
export GAZEBO_MODEL_PATH=${PROVANT_DATABASE}/models/
echo "export GAZEBO_MODEL_PATH=${PROVANT_DATABASE}/models" >> ${HOME}/.bashrc
export DIR_ROS=${HOME}/catkin_ws
echo "export DIR_ROS=${HOME}/catkin_ws" >> ${HOME}/.bashrc

# Enter ROS workspace folder
cd catkin_ws
# Compile the packages in the ROS workspace
catkin_make

# Compile the ProVANT Simulator GUI
mkdir -p ${TILT_PROJECT}/source/build
cd ${TILT_PROJECT}/source/build 
qtchooser -qt=5 -run-tool=qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make

# Crate symbolic link to allow launching of the GUI from the terminal
sudo ln -s ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui

