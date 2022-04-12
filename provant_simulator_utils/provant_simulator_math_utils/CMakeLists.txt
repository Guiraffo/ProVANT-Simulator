# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_math_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
find_package(
  Eigen3
  3.3
  REQUIRED
  NO_MODULE
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  # CATKIN_DEPENDS
  DEPENDS
  eigen
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/integrator.cpp src/saturator.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

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

# Tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    integrator_tests
    tests/integrator_test.cpp
    WORKING_DIRECTORY
    ${PROJECT_SOURCE_DIR}/tests
  )
  target_link_libraries(
    integrator_tests
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    Eigen3::Eigen
  )

  catkin_add_gtest(
    saturator_tests
    tests/saturator_test.cpp
    WORKING_DIRECTORY
    ${PROJECT_SOURCE_DIR}/tests
  )
  target_link_libraries(saturator_tests ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
