cmake_minimum_required(VERSION 2.8.3)
project(teste)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
simulator_msgs)

catkin_package()
include_directories(include)
INCLUDE_DIRECTORIES (/usr/include/eigen3)
include_directories( ${catkin_INCLUDE_DIRS})
include_directories($ENV{TILT_PROJECT}/source/Structure/control_strategies/)

add_library(teste src/main.cpp)
target_link_libraries(teste ${catkin_LIBRARIES})
install(TARGETS
teste
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
