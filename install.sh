#!/bin/bash
# Entrando na pasta Home
cd ~

#criando variáveis de ambiente locais
export PROVANT_ROS=${HOME}/catkin_ws/src
echo "export PROVANT_ROS=${HOME}/catkin_ws/src" >> ${HOME}/.bashrc
export TILT_STRATEGIES=${HOME}/catkin_ws/devel/lib/
echo "export TILT_STRATEGIES=${HOME}/catkin_ws/devel/lib/" >> ${HOME}/.bashrc
export TILT_PROJECT=${PROVANT_ROS}/ProVANT-Simulator
echo "export TILT_PROJECT=${PROVANT_ROS}/ProVANT-Simulator" >> ${HOME}/.bashrc
export TILT_MATLAB=${TILT_PROJECT}/source/Structure/Matlab/
echo "export TILT_MATLAB=${TILT_PROJECT}/source/Structure/Matlab/" >> ${HOME}/.bashrc
export PROVANT_DATABASE=${TILT_PROJECT}/source/Database
echo "export PROVANT_DATABASE=${TILT_PROJECT}/source/Database" >> ${HOME}/.bashrc
export GAZEBO_MODEL_PATH=${PROVANT_DATABASE}/models/
echo "export GAZEBO_MODEL_PATH=${PROVANT_DATABASE}/models" >> ${HOME}/.bashrc
export DIR_ROS=${HOME}/catkin_ws
echo "export DIR_ROS=${HOME}/catkin_ws" >> ${HOME}/.bashrc

# Entrando no espaço de trabalho do ROS
cd catkin_ws
# Compilando estruturas de mensagem utilizadas na simulação utilizando controlador implementado em software local 
catkin_make --pkg simulator_msgs
#Compilando demais pacotes existentes no ambiente de trabalho do ROS
catkin_make

# Apontando computador para biblioteca qt5 caso haja outras versão de qt
export QTDIR=/usr/share/qt5

# Compilando código da interface gráfica
mkdir -p ${TILT_PROJECT}/source/build
cd ${TILT_PROJECT}/source/build 
qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make

# criando link simbólico para fácil execução do ProVANT Simulator
sudo ln -s ${TILT_PROJECT}/source/build/GUI /usr/local/bin/provant_gui

