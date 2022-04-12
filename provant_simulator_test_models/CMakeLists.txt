# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_test_models)

find_package(catkin REQUIRED)

catkin_package(# INCLUDE_DIRS include CATKIN_DEPENDS other_catkin_pkg DEPENDS system_lib
)

# cmake-format: off
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# cmake-format: on
