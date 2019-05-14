cmake_minimum_required(VERSION 2.8.3)
project(custom_plugins)

find_package(catkin REQUIRED COMPONENTS 
		roscpp 
		std_msgs 
		geometry_msgs 
		gazebo_ros
		simulator_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

#find_package(TinyXML REQUIRED) 
#include_directories("${TinyXML_INCLUDE_DIRS}")



#include_directories(plugins/<nome_da_pasta_do_plugin>)

include_directories(plugins/force)
include_directories(plugins/torque)
include_directories(plugins/imu)
include_directories(plugins/gps)
include_directories(plugins/magnetometer)
include_directories(plugins/servo)
include_directories(plugins/brushless)
include_directories(plugins/reset)
include_directories(plugins/temperature)
include_directories(plugins/sonar)
include_directories(plugins/step)
include_directories(plugins/update_timer)
include_directories(plugins/state_space)
include_directories(plugins/state_space2)
include_directories(plugins/state_space_load)
include_directories(plugins/UniversalLinkSensor)
include_directories(plugins/UniversalJointSensor)
include_directories(plugins/pertubation)
include_directories(plugins/aerodinamica)
include_directories(plugins/servo2)
include_directories(plugins/HIL_server)
include_directories(plugins/teste_plugin)
include_directories(plugins/get_angle_plugin)
include_directories(plugins/QuadData)
include_directories(plugins/Aerodinamica4dot0)







find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)




generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs 
   geometry_msgs
)



catkin_package(
    #DEPENDS gazebo 
    #CATKIN_DEPENDS roscpp std_msgs geometry_msgs 
    #INCLUDE_DIRS include
    #LIBRARIES
   #  INCLUDE_DIRS include
  	# CATKIN_DEPENDS roscpp
   	# DEPENDS  TinyXML
)

add_library(gazebo_ros_force plugins/force/force.cpp)
target_link_libraries(gazebo_ros_force ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_torque plugins/torque/torque.cpp)
target_link_libraries(gazebo_ros_torque ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_reset_plugin plugins/reset/reset_plugin.cpp)
target_link_libraries(gazebo_ros_reset_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_imu plugins/imu/imu.cpp)
target_link_libraries(gazebo_ros_imu ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_temperature plugins/temperature/temperature.cpp)
target_link_libraries(gazebo_ros_temperature ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_gps plugins/gps/gps.cpp)
target_link_libraries(gazebo_ros_gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_sonar plugins/sonar/sonar.cpp)
target_link_libraries(gazebo_ros_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_servo_motor_plugin plugins/servo/servo_motor_plug.cpp)
target_link_libraries(gazebo_ros_servo_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_brushless_plugin plugins/brushless/aerodinamica.cpp)
target_link_libraries(gazebo_ros_brushless_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_world_plugin plugins/step/tilt_world_plugin.cpp)
target_link_libraries(gazebo_ros_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_AllData_plugin plugins/state_space/AllData.cpp)
target_link_libraries(gazebo_ros_AllData_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_AllData_plugin2 plugins/state_space_load/AllData2.cpp)
target_link_libraries(gazebo_ros_AllData_plugin2 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_magnetometer plugins/magnetometer/magnetometer.cpp)
target_link_libraries(gazebo_ros_magnetometer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_universallink plugins/UniversalLinkSensor/UniversalLinkSensor.cpp)
target_link_libraries(gazebo_ros_universallink ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_universaljoint plugins/UniversalJointSensor/UniversalJointSensor.cpp)
target_link_libraries(gazebo_ros_universaljoint ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_pertubation plugins/pertubation/pertubation.cpp)
target_link_libraries(gazebo_ros_pertubation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_aerodinamica2_plugin plugins/aerodinamica/aerodinamica2.cpp)
target_link_libraries(gazebo_ros_aerodinamica2_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_AllData3_plugin plugins/state_space2/AllData5.cpp)
target_link_libraries(gazebo_ros_AllData3_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_servo2_motor_plugin plugins/servo2/servo2_motor_plug.cpp)
target_link_libraries(gazebo_ros_servo2_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_hil_server4 plugins/HIL_server/server.cpp plugins/HIL_server/protocol.cpp plugins/HIL_server/frame.cpp plugins/HIL_server/serial.cpp) 
target_link_libraries(gazebo_hil_server4 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(testePlugin plugins/teste_plugin/teste_plugin.cpp) 
target_link_libraries(testePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(AnglePlugin plugins/get_angle_plugin/get_angle_plugin.cpp) 
target_link_libraries(AnglePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_QuadData plugins/QuadData/QuadData.cpp) 
target_link_libraries(gazebo_ros_QuadData ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_Aerodinamica4dot0_plugin plugins/Aerodinamica4dot0/Aerodinamica.cpp) 
target_link_libraries(gazebo_ros_Aerodinamica4dot0_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  gazebo_hil_server4
  gazebo_ros_aerodinamica2_plugin
  gazebo_ros_force
  gazebo_ros_torque
  gazebo_ros_pertubation
  gazebo_ros_brushless_plugin
  gazebo_ros_reset_plugin
  gazebo_ros_imu
  gazebo_ros_magnetometer
  gazebo_ros_temperature
  gazebo_ros_gps
  gazebo_ros_sonar
  gazebo_ros_servo_motor_plugin
  gazebo_ros_AllData_plugin
  gazebo_ros_AllData_plugin2
  gazebo_ros_world_plugin
  gazebo_ros_universaljoint
  gazebo_ros_universallink
  gazebo_ros_servo2_motor_plugin
  testePlugin
  AnglePlugin
  gazebo_ros_QuadData
  gazebo_ros_Aerodinamica4dot0_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
