# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS gazebo_ros
             geometry_msgs
             provant_simulator_cmake
             provant_simulator_csv_writer
             provant_simulator_log_utils
             provant_simulator_sdf_parser
             provant_simulator_xml_reader
             roscpp
             simulator_msgs
             std_msgs
             std_srvs
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

run_conan(conanfile.txt)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_definitions(${spdlog_libspdlog_DEFINITIONS})
add_compile_definitions("SPDLOG_FMT_EXTERNAL")

# Declare the list of plugin libraries that will be built
set(PLUGIN_LIBRARIES
    provant_config_plugin
    provant_external_forces_sensor_plugin
    provant_lidar_closest_obj_plugin
    provant_pose_sensor
    provant_position_logger_plugin
    provant_step_service_plugin
)

set(PLUGIN_INCLUDE_DIRS include)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${spdlog_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
  include
)

catkin_package(
  INCLUDE_DIRS
  ${PLUGIN_INCLUDE_DIRS}
  LIBRARIES
  ${PLUGIN_LIBRARIES}
  CATKIN_DEPENDS
  gazebo_ros
  geometry_msgs
  provant_simulator_csv_writer
  provant_simulator_log_utils
  provant_simulator_sdf_parser
  provant_simulator_xml_reader
  roscpp
  simulator_msgs
  std_msgs
  std_srvs
  DEPENDS
  Boost
)

set(COMMON_SRCS)

# Please do this for every plugin provant_external_forces_sensor_plugin
add_library(
  provant_external_forces_sensor_plugin ${COMMON_SRCS} src/external_forces_sensor/external_forces_sensor.cpp
                                        src/external_forces_sensor/external_forces_sensor_ros.cpp
)
add_dependencies(provant_external_forces_sensor_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_external_forces_sensor_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(provant_config_plugin src/config_plugin/config_plugin.cpp ${COMMON_SRCS})
add_dependencies(provant_config_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_config_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(provant_pose_sensor src/pose_sensor/pose_sensor.cpp ${COMMON_SRCS})
add_dependencies(provant_pose_sensor ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_pose_sensor
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(provant_step_service_plugin src/step_service/step_service.cpp ${COMMON_SRCS})
add_dependencies(provant_step_service_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(provant_step_service_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(provant_position_logger_plugin src/position_logger_plugin/position_logger.cpp)
add_dependencies(provant_position_logger_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_position_logger_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${spdlog_LIBRARIES}
  ${fmt_LIBRARIES}
)

add_library(
  provant_lidar_closest_obj_plugin src/lidar_closest_detection_plugin/closest_point_finder.cpp
                                   src/lidar_closest_detection_plugin/lidar_closest_obj_ros.cpp
)
add_dependencies(provant_lidar_closest_obj_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_lidar_closest_obj_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${spdlog_LIBRARIES}
  ${fmt_LIBRARIES}
  GpuRayPlugin
)
# Necessary to enable the linker to find the GpuRayPlugin library
target_link_directories(provant_lidar_closest_obj_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(
  TARGETS ${PLUGIN_LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# cmake-format: off
# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_provant_simulator_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test) target_link_libraries(${PROJECT_NAME}-test
# ${PROJECT_NAME}) endif()
# cmake-format: on
