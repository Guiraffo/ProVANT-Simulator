# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(vant3_adaptive_mix_ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulator_msgs
  control_strategies_base
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
	# INCLUDE_DIRS
	LIBRARIES
		${PROJECT_NAME}
	CATKIN_DEPENDS
		roscpp
		std_msgs
    simulator_msgs
    control_strategies_base
	# DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/main.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

install(TARGETS
  ${PROJECT_NAME}

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
