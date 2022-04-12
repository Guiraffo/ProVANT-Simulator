# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

# The dump_cmake_variables functions can print he values of cmake variables to the output screen. This is a debug
# function, please remove it or comment the calls to this method before commiting.
#
# Usage:
#
# * dump_cmake_variables() print the values of all CMake variables.
# * dump_cmake_variables(CATKIN) print the values of the CMake variables matching the "CATKIN" regular expression.
function(dump_cmake_variables)
  get_cmake_property(_variableNames VARIABLES)
  list(SORT _variableNames)
  if(ARGV0)
    message(STATUS "Printing out the CMake variables matching the \"${ARGV0}\" regular expression:")
  else()
    message(STATUS "Printing out CMake variables:")
  endif()
  foreach(_variableName ${_variableNames})
    if(ARGV0)
      unset(MATCHED)
      string(
        REGEX MATCH
              ${ARGV0}
              MATCHED
              ${_variableName}
      )
      if(NOT MATCHED)
        continue()
      endif()
    endif()
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endfunction()
