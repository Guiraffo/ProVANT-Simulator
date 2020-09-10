#!/bin/bash
#
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
#

# Stop execution if any command fail
set -e

# Print debugging messages
echo -e "\e[92mStarting ProVANT-Simulator compilation process\e[0m"

# Print an error message informing which command failed
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo -e "\e[91m\"${last_command}\" command failed with exit code $?.\e[0m"' EXIT

# Source .bashrc script
source $HOME/.bashrc

# Print environment variables
echo "Compiling the project with the following environment variables values"
echo "\$PROVANT_ROS = $PROVANT_ROS"
echo "\$TILT_STRATEGIES = $TILT_STRATEGIES"
echo "\$TILT_PROJECT = $TILT_PROJECT"
echo "\$TILT_MATLAB = $TILT_MATLAB"
echo "\$PROVANT_DATABASE = $PROVANT_DATABASE"
echo "\${GAZEBO_MODEL_PATH} = ${GAZEBO_MODEL_PATH}"
echo "\$DIR_ROS = $DIR_ROS"

echo -e "\nEnvironment variables resolved paths"
echo "\$PROVANT_ROS = `cd $PROVANT_ROS;pwd`"
echo "\$TILT_STRATEGIES = `cd $TILT_STRATEGIES;pwd`"
echo "\$TILT_PROJECT = `cd $TILT_PROJECT;pwd`"
echo "\$TILT_MATLAB = `cd $TILT_MATLAB;pwd`"
echo "\$PROVANT_DATABASE = `cd $PROVANT_DATABASE;pwd`"
echo "\$DIR_ROS = `cd $DIR_ROS;pwd`"

echo -e "\n"

# Enter ROS Workspace and compile the existing packages
echo -e "\e[92mStarting compilation of the ROS workspace packages\e[0m"
catkin_make --directory ${DIR_ROS}
echo -e "\e[92mCompilation of the ROS workspace packages finished with success.\e[0m"

# Compiling the GUI
echo -e "\e[92mStarting GUI compilation\e[0m"
cd ${TILT_PROJECT}/source/build 
qtchooser -qt=5 -run-tool=qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make
echo -e "\e[92mGUI compilation finished with success\e[0m"

# Update the symbolic link to allow launching of the GUI from the terminal
echo -e "\e[92mUpdating the symbolic link\e[0m"
sudo ln -sf ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui

echo -e "\e[94mCompilation finished!\e[0m"

# Remove EXIT trap if everything worked fine
trap - EXIT
