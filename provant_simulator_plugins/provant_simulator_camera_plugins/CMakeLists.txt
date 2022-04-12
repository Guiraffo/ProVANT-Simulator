# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_camera_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# Silence warning about unquoted variables in find_boost
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS 
    gazebo_dev            
  gazebo_plugins
             gazebo_ros
             provant_simulator_cmake
             provant_simulator_log_utils
             provant_simulator_sdf_parser
             roscpp
)

find_package(OpenCV REQUIRED)
run_conan(conanfile.txt)
find_package(spdlog REQUIRED)
find_package(
  Qt5
  COMPONENTS Core Widgets
  REQUIRED
)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  provant_camera_recorder
  provant_sync_camera_manager
  provant_sync_camera_publisher
  provant_sync_camera_rec
  provant_user_camera_follower
  provant_user_camera_rec
  CATKIN_DEPENDS
  gazebo_dev
  gazebo_plugins
  gazebo_ros
  provant_simulator_log_utils
  provant_simulator_sdf_parser
  roscpp
  DEPENDS
  OpenCV
)

add_definitions(${spdlog_libspdlog_DEFINITIONS})
add_compile_definitions("SPDLOG_FMT_EXTERNAL")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS}
  ${spdlog_INCLUDE_DIRS}
)

add_library(
  provant_camera_recorder STATIC src/camera_recorder/camera_recorder.cpp
                                 src/camera_recorder/recorder_options_parser.cpp
)
add_dependencies(provant_camera_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_camera_recorder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${spdlog_LIBRARIES}
)
set_property(TARGET provant_camera_recorder PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(provant_user_camera_follower src/user_camera_follower/user_camera_follower.cpp)
add_dependencies(provant_user_camera_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_user_camera_follower
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(provant_sync_camera_manager src/sync_camera_manager/sync_camera_manager.cpp)
add_dependencies(provant_sync_camera_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_sync_camera_manager
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

set(USER_CAMERA_REC_SOURCES src/user_camera_sync_recorder/user_camera_sync_recorder.cpp)
qt5_wrap_cpp(SOURCES src/user_camera_sync_recorder/user_camera_sync_recorder.h)
add_library(provant_user_camera_rec ${SOURCES})
add_dependencies(provant_user_camera_rec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_user_camera_rec
  provant_camera_recorder
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)

add_library(provant_sync_camera_rec src/camera_sync_recorder/camera_sync_recorder.cpp)
add_dependencies(provant_sync_camera_rec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_sync_camera_rec
  provant_camera_recorder
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(provant_sync_camera_publisher src/camera_sync_publisher/camera_sync_publisher.cpp)
add_dependencies(provant_sync_camera_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  provant_sync_camera_publisher
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  CameraPlugin
)
# Necessary to enable the linker to find the CameraPlugin library
target_link_directories(provant_sync_camera_publisher PRIVATE ${GAZEBO_LIBRARY_DIRS})

install(
  TARGETS provant_camera_recorder
          provant_sync_camera_manager
          provant_sync_camera_publisher
          provant_sync_camera_rec
          provant_user_camera_follower
          provant_user_camera_rec
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
