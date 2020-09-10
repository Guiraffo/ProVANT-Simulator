#!/bin/bash
#
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
#

# Stop execution if any command fail
set -e

# Print debugging messages
echo -e "\e[92mStarting ProVANT-Simulator installation process\e[0m"

# Print an error message informing which command failed
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo -e "\e[91m\"${last_command}\" command failed with exit code $?.\e[0m"' EXIT

# Get current dir
CUR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Echo current directory
echo "Running install.sh script from the directory $CUR_DIR"

# Source .bashrc script
source $HOME/.bashrc

echo -e "\e[92mChecking and creating environment variables\e[0m"

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
PROVANT_MODELS_PATH=$CUR_DIR/source/Database/models

if [[ ${GAZEBO_MODEL_PATH} == *$PROVANT_MODELS_PATH* ]]
then
    echo ""
else
    # This variable should point to the folder containing the Gazebo models distributed with the simulator
    # For example ${HOME}/catkin_ws/src/ProVANT_Simulator/source/Database/models/
    
    gazebo_model_str=`echo $GAZEBO_MODEL_PATH | tr : " "`
    read -r -d '' -a gz_paths < <(printf '%s\0' "$gazebo_model_str")
    declare -A SeenPaths
    UniquePaths=()
    for w in "${gz_paths[@]}"; do
        [[ ${SeenPaths[$w]} ]] && continue
        UniquePaths+=( "$w" )
        SeenPaths[$w]=x
    done
    IFS=: eval 'GAZEBO_MODEL_PATH="${UniquePaths[*]}":'
    
    echo '# Ensure that GAZEBO_MODEL_PATH has only unique paths' >> ${HOME}/.bashrc
    echo 'gazebo_model_str=`echo $GAZEBO_MODEL_PATH | tr : " "`' >> ${HOME}/.bashrc
    echo 'read -r -d '"''"' -a gz_paths < <(printf '"'"'%s\0'"'"' "$gazebo_model_str")' >> ${HOME}/.bashrc
    echo 'declare -A SeenPaths' >> ${HOME}/.bashrc
    echo 'UniquePaths=()' >> ${HOME}/.bashrc
    echo 'for w in "${gz_paths[@]}"; do' >> ${HOME}/.bashrc
    echo -e '\t[[ ${SeenPaths[$w]} ]] && continue' >> ${HOME}/.bashrc
    echo -e '\tUniquePaths+=( "$w" )' >> ${HOME}/.bashrc
    echo -e '\tSeenPaths[$w]=x' >> ${HOME}/.bashrc
    echo 'done' >> ${HOME}/.bashrc
    echo -e 'IFS=: eval '"'"'GAZEBO_MODEL_PATH="${UniquePaths[*]}":'"'" >> ${HOME}/.bashrc
    
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

# Print environment variables
echo "Compiling the project with the following environment variables values"
echo "\$PROVANT_ROS = $PROVANT_ROS"
echo "\$TILT_STRATEGIES = $TILT_STRATEGIES"
echo "\$TILT_PROJECT = $TILT_PROJECT"
echo "\$TILT_MATLAB = $TILT_MATLAB"
echo "\$PROVANT_DATABASE = $PROVANT_DATABASE"
echo "\$GAZEBO_MODEL_PATH = ${GAZEBO_MODEL_PATH}"
echo "\$DIR_ROS = $DIR_ROS"

echo -e "\nEnvironment variables resolved paths"
echo "\$PROVANT_ROS = `cd $PROVANT_ROS;pwd`"
echo "\$TILT_STRATEGIES = `cd $TILT_STRATEGIES;pwd`"
echo "\$TILT_PROJECT = `cd $TILT_PROJECT;pwd`"
echo "\$TILT_MATLAB = `cd $TILT_MATLAB;pwd`"
echo "\$PROVANT_DATABASE = `cd $PROVANT_DATABASE;pwd`"
echo "\$DIR_ROS = `cd $DIR_ROS;pwd`"

echo -e "\n"

# Change directory to the current user home dir
cd ${HOME}

# Compile the packages in the ROS workspace
echo -e "\e[92mStarting compilation of the ROS workspace packages\e[0m"
catkin_make --directory ${DIR_ROS}
echo -e "\e[92mCompilation of the ROS workspace packages finished with success.\e[0m"

# Compile the ProVANT Simulator GUI
echo -e "\e[92mStarting GUI compilation\e[0m"
mkdir -p ${TILT_PROJECT}/source/build
cd ${TILT_PROJECT}/source/build 
qtchooser -qt=5 -run-tool=qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make
echo -e "\e[92mGUI compilation finished successfully.\e[0m"

# Create symbolic link to allow launching of the GUI from the terminal
echo -e "\e[92mCreating symbolic link to the GUI\e[0m"
sudo ln -sf ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui

echo -e "\e[94mInstallation finished! Have fun!\e[0m"

# Remove EXIT trap if everything worked fine
trap - EXIT
