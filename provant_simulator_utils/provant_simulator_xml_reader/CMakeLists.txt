# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)
project(provant_simulator_xml_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)
# The cmake_modules package was not behaving, so we added its find package twice.
find_package(cmake_modules REQUIRED)
find_package(TinyXML2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
  # CATKIN_DEPENDS
  #   # cmake_modules
  DEPENDS
    TinyXML2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
  src/config_reader.cpp
)
add_dependencies(${PROJECT_NAME} 
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML2_LIBRARIES}
)
# Set the -fPIC linker option
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

install(TARGETS 
  ${PROJECT_NAME}

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME} tests/config_reader_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  )
  # Silence deprecation warnings
  set_target_properties(test_${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
  target_link_libraries(test_${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${TinyXML2_LIBRARIES}
    ${PROJECT_NAME}
  )
endif()
