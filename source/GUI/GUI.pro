#-------------------------------------------------
#
# Project created by QtCreator 2017-03-02T18:27:50
#
#-------------------------------------------------

QT       += core gui
QT       += xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GUI
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    DataAccess/ControllerElements/configfile.cpp \
    DataAccess/GazeboElements/Items/include_da.cpp \
    DataAccess/GazeboElements/Items/plugin_da.cpp \
    DataAccess/GazeboElements/Items/physics_da.cpp \
    DataAccess/GazeboElements/Items/gravity_da.cpp \
    DataAccess/GazeboElements/Items/link_da.cpp \
    DataAccess/GazeboElements/Items/joint_da.cpp \
    Business/model.cpp \
    Business/world.cpp \
    Business/controller.cpp \
#    Business/simulation.cpp \
    DataAccess/GazeboElements/worldfile.cpp \
    DataAccess/GazeboElements/Items/model_da.cpp \
    DataAccess/GazeboElements/Items/worldplugin.cpp \
    DataAccess/GazeboElements/Items/modelplugin.cpp \
    DataAccess/GazeboElements/Items/geometrymesh.cpp \
    DataAccess/GazeboElements/Items/inertial.cpp \
    DataAccess/GazeboElements/Items/material.cpp \
    DataAccess/GazeboElements/Items/collision.cpp \
    DataAccess/GazeboElements/Items/visual.cpp \
    DataAccess/GazeboElements/modelfile.cpp \
    DataAccess/GazeboElements/Items/axis.cpp \
    Business/treeitens.cpp \
    dialog.cpp \
    dialognewcontroller.cpp \
    DataAccess/ControllerElements/newstrategy.cpp \
    dialogopencontroller.cpp \
    DataAccess/RosElements/roslaunch.cpp \
    dialognewmodel.cpp \
    DataAccess/GazeboElements/Items/scene.cpp \
#    DataAccess/GazeboElements/Items/Instruments/sensor.cpp \
#    DataAccess/GazeboElements/Items/Instruments/sonar.cpp \
#    DataAccess/GazeboElements/Items/Instruments/gps.cpp \
#    DataAccess/GazeboElements/Items/Instruments/imu.cpp \
#    DataAccess/GazeboElements/Items/Instruments/magnetometer.cpp \
#    aboutform.cpp \
    aboutdialog.cpp \
    DataAccess/GazeboElements/Items/multipleincludes.cpp \
    DataAccess/GazeboElements/Items/multipleplugins.cpp
    Utils/tools.cpp

HEADERS  += mainwindow.h \
    DataAccess/ControllerElements/configfile.h \
    DataAccess/GazeboElements/Items/include_da.h \
    DataAccess/GazeboElements/Items/plugin_da.h \
    DataAccess/GazeboElements/Items/physics_da.h \
    DataAccess/GazeboElements/Items/gravity_da.h \
    DataAccess/GazeboElements/Items/link_da.h \
    DataAccess/GazeboElements/Items/joint_da.h \
    Business/model.h \
    Business/world.h \
    Business/controller.h \
#    Business/simulation.h \
    DataAccess/GazeboElements/worldfile.h \
    DataAccess/GazeboElements/Items/model_da.h \
    DataAccess/GazeboElements/Items/worldplugin.h \
    DataAccess/GazeboElements/Items/modelplugin.h \
    DataAccess/GazeboElements/Items/geometry.h \
    DataAccess/GazeboElements/Items/geometrymesh.h \
    DataAccess/GazeboElements/Items/inertial.h \
    DataAccess/GazeboElements/Items/material.h \
    DataAccess/GazeboElements/Items/collision.h \
    DataAccess/GazeboElements/Items/visual.h \
    DataAccess/GazeboElements/modelfile.h \
    DataAccess/GazeboElements/Items/axis.h \
    Business/treeitens.h \
    dialog.h \
    dialognewcontroller.h \
    DataAccess/ControllerElements/newstrategy.h \
    dialogopencontroller.h \
    DataAccess/RosElements/roslaunch.h \
    dialognewmodel.h \
    DataAccess/GazeboElements/Items/scene.h \
#    DataAccess/GazeboElements/Items/Instruments/sensor.h \
#    aboutform.h \
    aboutdialog.h \
    DataAccess/GazeboElements/Items/multipleincludes.h \
    Utils/tools.h \
    Utils/environmentexception.h \
    DataAccess/GazeboElements/Items/multipleplugins.h

FORMS    += mainwindow.ui \
    dialog.ui \
    dialognewcontroller.ui \
    dialogopencontroller.ui \
    dialognewmodel.ui \
#    aboutform.ui \
    aboutdialog.ui
