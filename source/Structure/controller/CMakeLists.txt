# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  simulator_msgs
  control_strategies_base
  provant_simulator_xml_reader
  std_srvs
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  # LIBRARIES
  CATKIN_DEPENDS
    roscpp
    simulator_msgs
    control_strategies_base
    std_srvs
    provant_simulator_xml_reader
  DEPENDS 
    EIGEN3 
)

add_executable(controller src/nodes/controller_node.cpp src/controller.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  controller 
  ${catkin_LIBRARIES} 
  ${CMAKE_DL_LIBS}
  Eigen3::Eigen
)

install(TARGETS 
  controller
  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/controller
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
