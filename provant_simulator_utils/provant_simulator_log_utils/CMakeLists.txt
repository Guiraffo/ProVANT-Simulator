# This file is part of the ProVANT simulator project. Licensed under the terms
# of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.21)
project(provant_simulator_log_utils CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS gazebo_ros provant_simulator_cmake
                                        roscpp)

run_conan(conanfile.txt)
find_package(spdlog REQUIRED)
find_package(GTest REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  provant_ros_log
  provant_gz_log
  provant_logtest_plugin
  CATKIN_DEPENDS
  gazebo_ros
  roscpp
  # DEPENDS system_lib
)

add_definitions(${spdlog_libspdlog_DEFINITIONS})
add_compile_definitions("SPDLOG_FMT_EXTERNAL")

include_directories(
  include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${spdlog_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS} ${GTest_gtest_INCLUDE_DIRS})

add_library(provant_ros_log STATIC src/roslog/rossink.cpp)
add_dependencies(provant_ros_log ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(provant_ros_log ${catkin_LIBRARIES} ${spdlog_LIBRARIES}
                      ${fmt_LIBRARIES})
set_property(TARGET provant_ros_log PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(provant_gz_log STATIC src/gzlog/gzlog.cpp)
add_dependencies(provant_gz_log ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(provant_gz_log ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
                      ${spdlog_LIBRARIES} ${fmt_LIBRARIES})
set_property(TARGET provant_gz_log PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(provant_logtest_plugin src/test_plugin/logtestplugin.cpp)
add_dependencies(provant_logtest_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_logtest_plugin PUBLIC spdlog::spdlog provant_gz_log)
target_link_libraries(provant_logtest_plugin PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(
  TARGETS provant_gz_log provant_logtest_plugin provant_ros_log
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

# Install spdlog
# install(TARGETS ${spdlog_LIBRARIES}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
  
# install(
#   DIRECTORY ${spdlog_INCLUDE_DIRS}/spdlog/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
#   FILES_MATCHING 
#   PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# print_imported_targets()
# dump_cmake_variables(spdlog)

if(CATKIN_ENABLE_TESTING)
  # catkin_add_gtest(test_${PROJECT_NAME} tests/test.cpp WORKING_DIRECTORY
  # ${PROJECT_SOURCE_DIR}/tests ) target_link_libraries(test_${PROJECT_NAME}
  # ${catkin_LIBRARIES} ${PROJECT_NAME} )
endif()
