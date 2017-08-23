cmake_minimum_required(VERSION 2.8.3)
project(controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  simulator_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(TinyXML REQUIRED) 

include_directories( ${TinyXML_INCLUDE_DIRS})
include_directories($ENV{TILT_PROJECT}/source/Structure/control_strategies/)
message(" teste $ENV{TILT_PROJECT}/source/Structure/control_strategies/")

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES Tilt_controller
   CATKIN_DEPENDS roscpp
   DEPENDS Eigen TinyXML
)

add_executable(controller src/tilt_controller2.cpp )
add_executable(controller2 src/tilt_controller.cpp )
add_executable(server src/server.cpp )
target_link_libraries(controller ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(controller2 ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(server ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${CMAKE_DL_LIBS})





