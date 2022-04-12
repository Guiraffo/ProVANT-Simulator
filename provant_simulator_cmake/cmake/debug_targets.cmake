# https://stackoverflow.com/questions/32183975/how-to-print-all-the-properties-of-a-target-in-cmake/56738858#56738858
# https://stackoverflow.com/a/56738858/3743145

# Get all properties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)
# Convert command output into a CMake list
string(
  REGEX
  REPLACE ";"
          "\\\\;"
          CMAKE_PROPERTY_LIST
          "${CMAKE_PROPERTY_LIST}"
)
string(
  REGEX
  REPLACE "\n"
          ";"
          CMAKE_PROPERTY_LIST
          "${CMAKE_PROPERTY_LIST}"
)

list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)

# cmake-format: off
# A function that prints all of the properties of a CMake target
# Parameters:
# - tgt: Target to print the information.
# cmake-format: on
function(print_target_properties tgt)
  if(NOT TARGET ${tgt})
    message("There is no target named '${tgt}'")
    return()
  endif()

  foreach(prop ${CMAKE_PROPERTY_LIST})
    string(
      REPLACE "<CONFIG>"
              "${CMAKE_BUILD_TYPE}"
              prop
              ${prop}
    )
    get_target_property(propval ${tgt} ${prop})
    if(propval)
      message("${tgt} ${prop} = ${propval}")
    endif()
  endforeach(prop)
endfunction(print_target_properties)

# cmake-format: off
# Print a list of the imported targets available in the current context.
# Note: Requires CMake 3.21+
# cmake-format: on
function(print_imported_targets)
  get_directory_property(imp_targets IMPORTED_TARGETS)
  list(SORT imp_targets)
  message(STATUS "Value of IMPORTED_TARGETS: ${imp_targets}")
  message(STATUS "Printing out all Imported CMake targets:")
  foreach(_tgt ${imp_targets})
    if(TARGET ${_tgt})
      message("\t-- ${_tgt}")
    endif()
  endforeach()
endfunction()
