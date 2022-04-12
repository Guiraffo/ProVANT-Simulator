# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_cmake)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS
  conan.cmake
  debug_targets.cmake
  dump_variables.cmake
  ${PROJECT_NAME}-extras.cmake
)

install(
  DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
