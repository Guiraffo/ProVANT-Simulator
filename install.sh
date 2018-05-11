#!/bin/bash
cd ~

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

cd catkin_ws
catkin_make --pkg simulator_msgs
catkin_make

mkdir -p ${TILT_PROJECT}/source/build
cd ${TILT_PROJECT}/source/build
qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
if sudo ln -s ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui
then
	rm  /usr/local/bin/provant_gui
	sudo ln -s ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui
fi

# Change the name of directory from ProVANT-Simulator_Developer to ProVANT-Simulator before compiling
