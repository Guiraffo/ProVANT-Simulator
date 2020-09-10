# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  simulator_msgs
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(TinyXML REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  "$ENV{TILT_PROJECT}/source/Structure/control_strategies/"
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES Tilt_controller
   CATKIN_DEPENDS roscpp
   DEPENDS EIGEN3 TinyXML
)

add_executable(controller src/nodes/controller_node.cpp src/controller.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  controller 
  ${catkin_LIBRARIES} 
  ${TinyXML_LIBRARIES} 
  ${CMAKE_DL_LIBS}
  Eigen3::Eigen
)

install(TARGETS controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
