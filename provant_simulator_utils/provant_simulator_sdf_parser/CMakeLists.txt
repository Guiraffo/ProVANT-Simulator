# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.14)
project(provant_simulator_sdf_parser)

# Silence warning about unquoted variables in find_boost
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS gazebo_ros provant_simulator_cmake provant_simulator_parser_utils)
find_package(gazebo REQUIRED)

# Provide the Microsoft C++ Guidelines Support Library
run_conan(${CMAKE_CURRENT_SOURCE_DIR}/conan.txt)
dump_cmake_variables(CONAN)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  gazebo_ros
  provant_simulator_parser_utils
  # DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${CONAN_INCLUDE_DIRS_MS-GSL}
)

add_library(${PROJECT_NAME} STATIC src/sdf_parser.cpp src/sdf_status.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    test_${PROJECT_NAME}
    tests/sdf_reader_test.cpp
    WORKING_DIRECTORY
    ${PROJECT_SOURCE_DIR}/tests
  )
  target_link_libraries(
    test_${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${GAZEBO_LIBRARIES}
  )
  catkin_add_gtest(
    test_sdf_status
    tests/sdf_status_tests.cpp
    WORKING_DIRECTORY
    ${PROJECT_SOURCE_DIR}/tests
  )
  target_link_libraries(
    test_sdf_status
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${GAZEBO_LIBRARIES}
  )
endif()
