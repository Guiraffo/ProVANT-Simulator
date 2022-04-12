# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_resource_compiler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS cmakerc.cmake)

if(CATKIN_ENABLE_TESTING)
  include(cmake/cmakerc.cmake)
  add_cmakerc()

  cmrc_add_resource_library(
    testrcs-resources
    ALIAS
    testrcs::rc
    NAMESPACE
    testrcs
    tests/resources/test1.txt
  )

  catkin_add_gtest(
    test_${PROJECT_NAME}
    tests/main.cpp
    WORKING_DIRECTORY
    ${PROJECT_SOURCE_DIR}/tests
  )
  target_link_libraries(test_${PROJECT_NAME} testrcs::rc)
endif()
