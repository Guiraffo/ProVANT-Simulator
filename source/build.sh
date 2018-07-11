#!/bin/bash
cd ~

export QTDIR=/usr/share/qt5

mkdir -p ${TILT_PROJECT}/source/build
cd ${TILT_PROJECT}/source/build 
qmake ${TILT_PROJECT}/source/GUI/GUI.pro -r -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make


