# This file is part of the ProVANT simulator project. Licensed under the terms
# of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(simulator_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES Sensor.msg SensorArray.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

# cmake-format: off
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
# cmake-format: on

# cmake-format: off
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
# cmake-format: on

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
