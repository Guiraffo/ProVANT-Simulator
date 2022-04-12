# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

message(STATUS "Finding CPLEX")

if(DEFINED ENV{CPLEX_PATH})
  message("Loading the CPLEX path from the CPLEX_PATH environment variable with value: $ENV{CPLEX_PATH}")
  set(CPLEX_BASE_DIR $ENV{CPLEX_PATH})
endif()

include(FindPackageHandleStandardArgs)

if(UNIX)
  set(CPLEX_ILOG_DIRS /opt/ibm/ILOG /opt/IBM/ILOG)

  # Find the arch of the current machine
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CPLEX_ARCH x86-64)
  else()
    set(CPLEX_ARCH x86)
  endif()

  set(CPLEX_LIB_PATH_SUFFIXES lib/${CPLEX_ARCH}_sles10_4.1/static_pic lib/${CPLEX_ARCH}_linux/static_pic)
endif()

if(NOT CPLEX_STUDIO_DIR)
  foreach(dir ${CPLEX_ILOG_DIRS})
    file(GLOB CPLEX_STUDIO_DIRS "${dir}/CPLEX_Studio*")
    list(SORT CPLEX_STUDIO_DIRS)
    list(REVERSE CPLEX_STUDIO_DIRS)
    message("Found studio dirs: ${CPLEX_STUDIO_DIRS}")
    if(CPLEX_STUDIO_DIRS)
      list(
        GET
        CPLEX_STUDIO_DIRS
        0
        CPLEX_STUDIO_DIR_
      )
      string(
        REGEX MATCH
              "[0-9][0-9][0-9][0-9]"
              CPXVERSION
              ${CPLEX_STUDIO_DIR_}
      )
      message(STATUS "Found CPLEX Studio: ${CPLEX_STUDIO_DIR_}")
      message(STATUS "Detected CPLEX version ${CPXVERSION}")
      set(CPLEX_STUDIO_FOUND true)
      break()
    endif()
  endforeach()
  if(NOT CPLEX_STUDIO_DIR_)
    set(CPLEX_STUDIO_DIR_ CPLEX_STUDIO_DIR-NOTFOUND)
    set(CPLEX_STUDIO_FOUND false)
  endif()
  set(CPLEX_STUDIO_DIR
      ${CPLEX_STUDIO_DIR_}
      CACHE PATH "Path to the CPLEX Studio directory"
  )
else()
  set(CPLEX_STUDIO_FOUND true)
endif()

if(CPLEX_STUDIO_FOUND)
  message(STATUS "Found CPLEX Studio at ${CPLEX_STUDIO_DIR}")
  find_package(Threads)
  set(CPLEX_DIR ${CPLEX_STUDIO_DIR}/cplex)

  find_path(CPLEX_INCLUDE_DIR ilcplex/cplex.h PATHS ${CPLEX_DIR}/include)
  find_library(
    CPLEX_LIBRARY
    NAMES cplex
    PATHS ${CPLEX_DIR}
    PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES}
  )
  find_library(
    ILOCPLEX_LIBRARY
    NAMES ilocplex
    PATHS ${CPLEX_DIR}
    PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES}
  )
  set(CPLEX_LIBRARY_DEBUG ${CPLEX_LIBRARY})

  set(CPLEX_CONCERT_DIR ${CPLEX_STUDIO_DIR}/concert)
  find_path(CPLEX_CONCERT_INCLUDE_DIR ilconcert/cplexconcertdoc.h PATHS ${CPLEX_CONCERT_DIR}/include)
  find_library(
    CPLEX_CONCERT_LIBRARY
    NAMES concert
    PATHS ${CPLEX_CONCERT_DIR}
    PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES}
  )

  set(CPLEX_LIBRARIES "${CPLEX_LIBRARY};${ILOCPLEX_LIBRARY}")
  set(CPLEX_EXTRA_LIBRARIES pthread m dl)
  set(CPLEX_COMPILER_DEFINITIONS IL_STD)

else()
  message(STATUS "CPLEX Studio was not found")
endif()

find_package_handle_standard_args(
  CPLEX
  DEFAULT_MSG
  CPLEX_LIBRARIES
  CPLEX_LIBRARIES
  CPLEX_INCLUDE_DIR
)

mark_as_advanced(CPLEX_LIBRARY CPLEX_LIBRARY_DEBUG CPLEX_INCLUDE_DIR)
