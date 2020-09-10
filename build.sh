#!/bin/bash
#
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
#
cd ~

# Enter ROS Workspace and compile the existing packages
cd catkin_ws
catkin_make

# Compiling the GUI
mkdir -p ${TILT_PROJECT}/source/build
cd ${TILT_PROJECT}/source/build 
qtchooser -qt=5 -run-tool=qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make

# Update the symbolic link to allow launching of the GUI from the terminal
sudo ln -sf ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui
