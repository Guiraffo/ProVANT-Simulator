#-------------------------------------------------
#
# Project created by QtCreator 2017-03-02T18:27:50
#
#-------------------------------------------------

CONFIG += qt
QT += gui
QT += core
QT += xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GUI
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
    Utils/appsettings.cpp \
    Widgets/filebrowserwidget.cpp \
    applicationsettingsdialog.cpp \
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
    DataAccess/GazeboElements/worldfile.cpp \
    DataAccess/GazeboElements/Items/model_da.cpp \
    DataAccess/GazeboElements/Items/modelplugin.cpp \
    DataAccess/GazeboElements/Items/geometrymesh.cpp \
    DataAccess/GazeboElements/Items/inertial.cpp \
    DataAccess/GazeboElements/Items/material.cpp \
    DataAccess/GazeboElements/Items/collision.cpp \
    DataAccess/GazeboElements/Items/visual.cpp \
    DataAccess/GazeboElements/modelfile.cpp \
    DataAccess/GazeboElements/Items/axis.cpp \
    Business/treeitens.cpp \
    modelsetupdialog.cpp \
    dialognewcontroller.cpp \
    DataAccess/ControllerElements/newstrategy.cpp \
    DataAccess/RosElements/roslaunch.cpp \
    dialognewmodel.cpp \
    DataAccess/GazeboElements/Items/scene.cpp \
    aboutdialog.cpp \
    DataAccess/GazeboElements/Items/multipleincludes.cpp \
    DataAccess/GazeboElements/Items/multipleplugins.cpp \
    jointsdialog.cpp \
    processoutputwindow.cpp \
    ProcessOutput/processoutputformatter.cpp \
    ProcessOutput/terminalcolortable.cpp

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
    DataAccess/GazeboElements/worldfile.h \
    DataAccess/GazeboElements/Items/model_da.h \
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
    Utils/appsettings.h \
    Widgets/filebrowserwidget.h \
    applicationsettingsdialog.h \
    dialognewcontroller.h \
    DataAccess/ControllerElements/newstrategy.h \
    DataAccess/RosElements/roslaunch.h \
    dialognewmodel.h \
    DataAccess/GazeboElements/Items/scene.h \
    aboutdialog.h \
    DataAccess/GazeboElements/Items/multipleincludes.h \
    DataAccess/GazeboElements/Items/multipleplugins.h \
    jointsdialog.h \
    modelsetupdialog.h \
    processoutputwindow.h \
    ProcessOutput/processoutputformatter.h \
    ProcessOutput/terminalcolortable.h

FORMS    += mainwindow.ui \
    Widgets/filebrowserwidget.ui \
    applicationsettingsdialog.ui \
    modelsetupdialog.ui \
    dialognewcontroller.ui \
    dialognewmodel.ui \
    aboutdialog.ui \
    jointsdialog.ui \
    jointsdialog.ui \
    processoutputwindow.ui

RESOURCES += \
    icons.qrc \
    resources.qrc
