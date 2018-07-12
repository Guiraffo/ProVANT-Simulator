#!/bin/bash
cd ~

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

