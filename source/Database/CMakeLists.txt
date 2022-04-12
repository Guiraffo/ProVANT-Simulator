# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(Database)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(# INCLUDE_DIRS include LIBRARIES tilt CATKIN_DEPENDS DEPENDS system_lib
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
