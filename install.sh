#!/bin/bash
#
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
#

# Get current dir
CUR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Source .bashrc script
source $HOME/.bashrc

# Append environment variables to the current user .bashrc file.
 
# Test if PROVANT_ROS is set, and creating it if it is not
if [ -z "$PROVANT_ROS" ]
then
    # This variable should point to ${HOME}/catkin_ws/src/
    export PROVANT_ROS=$CUR_DIR/../
    echo "export PROVANT_ROS=${PROVANT_ROS}" >> ${HOME}/.bashrc
    echo "Creating \$PROVANT_ROS environment variable with value $PROVANT_ROS"
fi

if [ -z "$TILT_STRATEGIES" ]
then
    # This variable should point to ${HOME}/catkin_ws/devel/lib/
    export TILT_STRATEGIES=$CUR_DIR/../../devel/lib/
    echo "export TILT_STRATEGIES=${TILT_STRATEGIES}" >> ${HOME}/.bashrc
    echo "Creating \$TILT_STRATEGIES environment variable with value $TILT_STRATEGIES"
fi

if [ -z "$TILT_PROJECT" ]
then
    # This variable should point to the root of the ProVANT simulator folder
    # For example ${HOME}/catkin_ws/src/ProVANT_Simulator/
    export TILT_PROJECT=$CUR_DIR/
    echo "export TILT_PROJECT=${TILT_PROJECT}" >> ${HOME}/.bashrc
    echo "Creating \$TILT_PROJECT environment variable with value $TILT_PROJECT"
fi

if [ -z "$TILT_MATLAB" ]
then
    # This variable should point to the folder were the simulation logs will be saved
    # For example ${HOME}/catkin_ws/src/ProVANT_Simulator/source/Structure/Matlab/
    export TILT_MATLAB=$CUR_DIR/source/Structure/Matlab/
    echo "export TILT_MATLAB=${TILT_MATLAB}" >> ${HOME}/.bashrc
    echo "Creating \$TILT_MATLAB environment variable with value $TILT_MATLAB"
fi

if [ -z "$PROVANT_DATABASE" ]
then
    # This variable should point to the folder containing the Database of models, worlds and scenarios distributed with the ProVANT Simulator
    # For example ${HOME}/catkin_ws/src/ProVANT_Simulator/source/Database/
    export PROVANT_DATABASE=$CUR_DIR/source/Database/
    echo "export PROVANT_DATABASE=${PROVANT_DATABASE}" >> ${HOME}/.bashrc
    echo "Creating \$PROVANT_DATABASE environment variable with value $PROVANT_DATABASE"
fi

# Verify if the models of the simulador are in the GAZEBO_MODEL_PATH
PROVANT_MODELS_PATH=$CUR_DIR/source/Database/models/

if [[ ${GAZEBO_MODEL_PATH} == *$PROVANT_MODELS_PATH* ]]
then
    echo ""
else
    # This variable should point to the folder containing the Gazebo models distributed with the simulator
    # For example ${HOME}/catkin_ws/src/ProVANT_Simulator/source/Database/models/
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH$PROVANT_MODELS_PATH:
    echo "export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}$PROVANT_MODELS_PATH:" >> ${HOME}/.bashrc
    echo "Creating \$GAZEBO_MODEL_PATH environment variable with value $GAZEBO_MODEL_PATH"
fi

if [ -z "$DIR_ROS" ]
then
    # This variable should point to the catkin_workspace folder
    # For example ${HOME}/catkin_ws/
    export DIR_ROS=$CUR_DIR/../../
    echo "export DIR_ROS=${DIR_ROS}" >> ${HOME}/.bashrc
    echo "Creating \$DIR_ROS environment variable with value $DIR_ROS"
fi

# Change directory to the current user home dir
cd ${HOME}

# Enter ROS workspace folder
#cd catkin_ws
# Compile the packages in the ROS workspace
#catkin_make

# Compile the ProVANT Simulator GUI
#mkdir -p ${TILT_PROJECT}/source/build
#cd ${TILT_PROJECT}/source/build 
#qtchooser -qt=5 -run-tool=qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
#make

# Crate symbolic link to allow launching of the GUI from the terminal
#sudo ln -sf ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui

