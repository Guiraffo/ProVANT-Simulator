# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.0.2)

project(vant4_winf_transition)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    simulator_msgs
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    "$ENV{TILT_PROJECT}/source/Structure/control_strategies/"
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        roscpp
        std_msgs
        simulator_msgs
    #DEPENDS
)

# Defines the C++ version as C++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

if(DEFINED CPLEXDIR)
else()
# Defines an option for the CPLEX base path
set(
    CPLEXBASEDIR
    $ENV{HOME}/catkin_ws/src/ProVANT-Simulator_Developer/third_party/ibm/ILOG/CPLEX_Studio1210/
    CACHE
    PATH
    "Path for the root directory of the Cplex installation"
    FORCE
)
endif() #DEFINED CPLEXDIR

# Test if the base path for the CPLEXDIR is found
if(EXISTS ${CPLEXBASEDIR})
    message(STATUS "Cplex found at " ${CPLEXDIR}, " building the " ${PROJECT_NAME} " control strategy")

    #Includes, links and compiler flags for CPLEX (based on the Makefile provided with the CPLEX examples)
    set(SYSTEM "x86-64_linux")
    set(LIBFORMAT "static_pic")
    
    set(CPLEXDIR "${CPLEXBASEDIR}/cplex")
    set(CONCERTDIR  "${CPLEXBASEDIR}/concert")
    set(CPLEXCCOPT "-m64 -O -fPIC -fno-strict-aliasing -fexceptions -DIL_STD")
    
    set(CPLEXLIBDIR "${CPLEXDIR}/lib/${SYSTEM}/${LIBFORMAT}")
    set(CONCERTLIBDIR "${CONCERTDIR}/lib/${SYSTEM}/${LIBFORMAT}")
    set(CPLEXLIBTARGETS ilocplex concert cplex m pthread dl)

    set(CPLEXINCDIR "${CPLEXDIR}/include")
    set(CONCERTINCDIR "${CONCERTDIR}/include")

    include_directories(
        ${CPLEXINCDIR}
        ${CONCERTINCDIR}
    )
    link_directories(
        ${CPLEXLIBDIR} ${CONCERTLIBDIR}
    )
    set(CMAKE_CXX_FLAGS "${CPLEXCCOPT} ${CMAKE_CXX_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")

    add_library(${PROJECT_NAME} 
        src/main.cpp 
        src/cplexutils.cpp
        src/inertia_matrix.cpp
        src/input_coupling.cpp
        src/input_coupling_aero.cpp
        src/skew_symmetric_matrix.cpp
        src/gravitational_vector.cpp
        src/coriollis_matrix.cpp
    )

    link_directories(
        ${CPLEXLIBDIR} 
        ${CONCERTLIBDIR}
    )

    target_link_libraries(${PROJECT_NAME} 
        ${catkin_LIBRARIES} 
        Eigen3::Eigen
        ${CPLEXLIBTARGETS}
    )

    #Padrao do simulador
    install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
    )

else()
    message(WARNING "Cplex was not found at " ${CPLEXDIR})
endif()
