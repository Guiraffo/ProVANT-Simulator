# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_gui_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Necessary for Qt plugins compilation
set(CMAKE_AUTOMOC ON)
# Enable Qt automoc on headers files with the .hh extension
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

# From the Qt documentation
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# Silence warning about unquoted variables in find_boost
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             gazebo_ros
             std_msgs
             std_srvs
             provant_simulator_sdf_parser
)

find_package(
  Qt5
  COMPONENTS Core Widgets
  REQUIRED
)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  src
  LIBRARIES
  provant_auto_stepping_plugin
  provant_grid_ctrl_plugin
  CATKIN_DEPENDS
  roscpp
  gazebo_ros
  std_srvs
  std_msgs
  provant_simulator_sdf_parser
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

# Add directories from Qt and Gazebo
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# provant_auto_stepping_plugin
add_library(
  provant_auto_stepping_plugin SHARED ${headers_MOC} src/auto_stepping/auto_stepping.cpp
                                      src/auto_stepping/resources.qrc
)
add_dependencies(provant_auto_stepping_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_auto_stepping_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${OGRE-Terrain_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

# provant_grid_ctrl_plugin
add_library(provant_grid_ctrl_plugin SHARED src/grid_ctrl/grid_ctrl.cpp)
add_dependencies(provant_grid_ctrl_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_grid_ctrl_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${OGRE-Terrain_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

install(
  TARGETS provant_auto_stepping_plugin provant_grid_ctrl_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
