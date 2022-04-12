# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_parser_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Silence warning about unquoted variables in find_boost
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  # CATKIN_DEPENDS
  DEPENDS
    Boost
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
  src/type_conversion.cpp
)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

install(TARGETS ${PROJECT_NAME}

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME} tests/type_conversion_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )
endif()
