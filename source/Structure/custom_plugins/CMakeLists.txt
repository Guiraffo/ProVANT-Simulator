# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(custom_plugins)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Silence warning about unquoted variables in find_boost
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             geometry_msgs
             gazebo_ros
             simulator_msgs
             nav_msgs
             visualization_msgs
             provant_simulator_xml_reader
             provant_simulator_sdf_parser
)
find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(
  Eigen3
  3.3
  REQUIRED
  NO_MODULE
)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# To add a new plugin, please follow these steps: 1) Add the new plugin to the LIBRARIES section of the catkin_package
# 2) Add an include directories directive for the new plugin 3) Add an add_library and target_link_libraries for the new
# plugin 4) Add the new plugin to the TARGETS section fo the install directive

catkin_package(
  # The include files exported by this project INCLUDE_DIRS include The libraries generated by the this package
  LIBRARIES
  gazebo_ros_reset_plugin
  gazebo_ros_imu
  gazebo_ros_temperature
  gazebo_ros_gps
  gazebo_ros_sonar
  gazebo_ros_servo_motor_plugin
  gazebo_ros_brushless_plugin
  gazebo_ros_world_plugin
  gazebo_ros_AllData_plugin
  gazebo_ros_AllData_plugin2
  gazebo_ros_magnetometer
  gazebo_ros_universallink
  gazebo_ros_universaljoint
  gazebo_ros_pertubation
  gazebo_ros_aerodinamica2_plugin
  gazebo_ros_AllData3_plugin
  gazebo_ros_servo2_motor_plugin
  gazebo_hil_server4
  gazebo_ros_QuadData
  gazebo_ros_Aerodinamica4dot0_plugin
  gazebo_ros_Aerodinamica4dot2_plugin
  gazebo_ros_PathPlotter
  gazebo_ros_DataSave
  gazebo_ros_QuadForces
  gazebo_ros_VisualPropellers
  gazebo_ros_DataSaveTiltRotor
  gazebo_ros_turbulance
  gazebo_ros_vant5Data
  gazebo_ros_aerodinamica5
  gazebo_ros_FullAerodV5
  gazebo_ros_ArmForces
  # ROS packages necessary to build the package
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_ros
  simulator_msgs
  nav_msgs
  visualization_msgs
  provant_simulator_xml_reader
  provant_simulator_sdf_parser
  # System libraries necessary to build the package
  DEPENDS
  Boost
  TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

# Add in this section the include folders of all the plugins that should be compiled following this format
# include_directories(plugins/<plugin_folder_name>)

include_directories(plugins/imu)
include_directories(plugins/gps)
include_directories(plugins/magnetometer)
include_directories(plugins/servo)
include_directories(plugins/brushless)
include_directories(plugins/reset)
include_directories(plugins/temperature)
include_directories(plugins/sonar)
include_directories(plugins/step)
include_directories(plugins/state_space)
include_directories(plugins/state_space2)
include_directories(plugins/state_space_load)
include_directories(plugins/UniversalLinkSensor)
include_directories(plugins/UniversalJointSensor)
include_directories(plugins/pertubation)
include_directories(plugins/aerodinamica)
include_directories(plugins/servo2)
include_directories(plugins/HIL_server)
include_directories(plugins/QuadData)
include_directories(plugins/Aerodinamica4dot0)
include_directories(plugins/Aerodinamica4dot2)
include_directories(plugins/PathPlotter)
include_directories(plugins/DataSave)
include_directories(plugins/QuadForces)
include_directories(plugins/VisualPropellers)
include_directories(plugins/DataSaveTiltRotor)
include_directories(plugins/Turbulance)
include_directories(plugins/vant5Data)
include_directories(plugins/aerodinamica5)
include_directories(plugins/FullAerodV5)
include_directories(plugins/ArmForces)



# For each plugin, add a library, and declare its linking dependencies

add_library(gazebo_ros_reset_plugin plugins/reset/reset_plugin.cpp)
add_dependencies(gazebo_ros_reset_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_reset_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_imu plugins/imu/imu.cpp)
add_dependencies(gazebo_ros_imu ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_imu ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_temperature plugins/temperature/temperature.cpp)
add_dependencies(gazebo_ros_temperature ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_temperature ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_gps plugins/gps/gps.cpp)
add_dependencies(gazebo_ros_gps ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_sonar plugins/sonar/sonar.cpp)
add_dependencies(gazebo_ros_sonar ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_servo_motor_plugin plugins/servo/servo_motor_plug.cpp)
add_dependencies(gazebo_ros_servo_motor_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_servo_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_brushless_plugin plugins/brushless/brushless.cpp)
add_dependencies(gazebo_ros_brushless_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_brushless_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_world_plugin plugins/step/step_plugin.cpp)
add_dependencies(gazebo_ros_world_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_AllData_plugin plugins/state_space/AllData.cpp)
add_dependencies(gazebo_ros_AllData_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_AllData_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_AllData_plugin2 plugins/state_space_load/AllData2.cpp)
add_dependencies(gazebo_ros_AllData_plugin2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_AllData_plugin2 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_magnetometer plugins/magnetometer/magnetometer.cpp)
add_dependencies(gazebo_ros_magnetometer ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_magnetometer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_universallink plugins/UniversalLinkSensor/UniversalLinkSensor.cpp)
add_dependencies(gazebo_ros_universallink ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_universallink ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_universaljoint plugins/UniversalJointSensor/UniversalJointSensor.cpp)
add_dependencies(gazebo_ros_universaljoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_universaljoint ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_pertubation plugins/pertubation/pertubation.cpp)
add_dependencies(gazebo_ros_pertubation ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_pertubation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_aerodinamica2_plugin plugins/aerodinamica/aerodinamica2.cpp)
add_dependencies(gazebo_ros_aerodinamica2_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_aerodinamica2_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_AllData3_plugin plugins/state_space2/AllData5.cpp)
add_dependencies(gazebo_ros_AllData3_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_AllData3_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_servo2_motor_plugin plugins/servo2/servo2_motor_plug.cpp)
add_dependencies(gazebo_ros_servo2_motor_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_servo2_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(
  gazebo_hil_server4
  plugins/HIL_server/frame.cpp
  plugins/HIL_server/protocol.cpp
  plugins/HIL_server/serial.cpp
  plugins/HIL_server/server.cpp
)
add_dependencies(gazebo_hil_server4 ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_hil_server4 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_QuadData plugins/QuadData/QuadData.cpp)
add_dependencies(gazebo_ros_QuadData ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_QuadData ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_Aerodinamica4dot0_plugin plugins/Aerodinamica4dot0/Aerodinamica.cpp)
add_dependencies(gazebo_ros_Aerodinamica4dot0_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_Aerodinamica4dot0_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_Aerodinamica4dot2_plugin plugins/Aerodinamica4dot2/Aerodinamica.cpp)
add_dependencies(gazebo_ros_Aerodinamica4dot2_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_Aerodinamica4dot2_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_PathPlotter plugins/PathPlotter/PathPlotter.cpp)
add_dependencies(gazebo_ros_PathPlotter ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_PathPlotter ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_DataSave plugins/DataSave/DataSave.cpp)
add_dependencies(gazebo_ros_DataSave ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_DataSave ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_QuadForces plugins/QuadForces/QuadForces.cpp)
add_dependencies(gazebo_ros_QuadForces ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_QuadForces ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_VisualPropellers plugins/VisualPropellers/VisualPropellers.cpp)
add_dependencies(gazebo_ros_VisualPropellers ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_VisualPropellers ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_DataSaveTiltRotor plugins/DataSaveTiltRotor/DataSaveTiltRotor.cpp)
add_dependencies(gazebo_ros_DataSaveTiltRotor ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_DataSaveTiltRotor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_turbulance plugins/Turbulance/turbulance.cpp)
add_dependencies(gazebo_ros_turbulance ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_turbulance ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_vant5Data plugins/vant5Data/vant5Data.cpp)
add_dependencies(gazebo_ros_vant5Data ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_vant5Data ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_aerodinamica5 plugins/aerodinamica5/aerodinamica5.cpp)
add_dependencies(gazebo_ros_aerodinamica5 ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_aerodinamica5 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_FullAerodV5 plugins/FullAerodV5/FullAerodV5.cpp)
add_dependencies(gazebo_ros_FullAerodV5 ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_FullAerodV5 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_ArmForces plugins/ArmForces/ArmForces.cpp)
add_dependencies(gazebo_ros_ArmForces ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_ArmForces ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



# Install target for the include files of the plugin
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install targets for the plugin libraries
install(
  TARGETS gazebo_ros_reset_plugin
          gazebo_ros_imu
          gazebo_ros_temperature
          gazebo_ros_gps
          gazebo_ros_sonar
          gazebo_ros_servo_motor_plugin
          gazebo_ros_brushless_plugin
          gazebo_ros_world_plugin
          gazebo_ros_AllData_plugin
          gazebo_ros_AllData_plugin2
          gazebo_ros_magnetometer
          gazebo_ros_universallink
          gazebo_ros_universaljoint
          gazebo_ros_pertubation
          gazebo_ros_aerodinamica2_plugin
          gazebo_ros_AllData3_plugin
          gazebo_ros_servo2_motor_plugin
          gazebo_hil_server4
          gazebo_ros_QuadData
          gazebo_ros_Aerodinamica4dot0_plugin
          gazebo_ros_Aerodinamica4dot2_plugin
          gazebo_ros_PathPlotter
          gazebo_ros_DataSave
          gazebo_ros_QuadForces
          gazebo_ros_VisualPropellers
          gazebo_ros_DataSaveTiltRotor
          gazebo_ros_turbulance
          gazebo_ros_vant5Data
          gazebo_ros_aerodinamica5
          gazebo_ros_FullAerodV5
          gazebo_ros_ArmForces
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
