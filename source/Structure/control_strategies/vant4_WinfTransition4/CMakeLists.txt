 
cmake_minimum_required(VERSION 2.8.3)
project(vant4_WinfTransition4)
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

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


#Includes, links and compiler flags for CPLEX (based on the Makefile provided with the CPLEX examples)
set(SYSTEM "x86-64_linux")
set(LIBFORMAT "static_pic")

#if(EXISTS "/opt/ibm/ILOG/CPLEX_Studio_Community129") 
 if(EXISTS "/nao/existe/esse/caminho")       
#set(CPLEXDIR "$ENV{HOME}/CPLEX/cplex")
#set(CONCERTDIR  "$ENV{HOME}/CPLEX/concert")
set(CPLEXDIR "/opt/ibm/ILOG/CPLEX_Studio_Community129/cplex")
set(CONCERTDIR  "/opt/ibm/ILOG/CPLEX_Studio_Community129/concert")
set(CPLEXCCOPT "-m64 -O -fPIC -fno-strict-aliasing -fexceptions -DIL_STD")

set(CPLEXLIBDIR "${CPLEXDIR}/lib/${SYSTEM}/${LIBFORMAT}")
set(CONCERTLIBDIR "${CONCERTDIR}/lib/${SYSTEM}/${LIBFORMAT}")
set(CPLEXLIBTARGETS ilocplex concert cplex m pthread dl)

set(CPLEXINCDIR "${CPLEXDIR}/include")
set(CONCERTINCDIR "${CONCERTDIR}/include")

include_directories(${CPLEXINCDIR})
include_directories(${CONCERTINCDIR})
link_directories(${CPLEXLIBDIR} ${CONCERTLIBDIR})
set(CMAKE_CXX_FLAGS "${CPLEXCCOPT} ${CMAKE_CXX_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")

add_library(vant4_WinfTransition4 src/main.cpp include/cplexutils.cpp)


link_directories(${CPLEXLIBDIR} ${CONCERTLIBDIR})
target_link_libraries(vant4_WinfTransition4 ${catkin_LIBRARIES} ${CPLEXLIBTARGETS})

#Padrao do simulador
install(TARGETS
vant4_WinfTransition4
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
MESSAGE(WARNING "CPLEX not found in path /opt/ibm/ILOG/CPLEX_Studio_Community129")
#endif(EXISTS "/opt/ibm/ILOG/CPLEX_Studio_Community129")
endif(EXISTS "/nao/existe/esse/caminho")
unset(EXISTS CACHE)
