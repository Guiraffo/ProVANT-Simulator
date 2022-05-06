# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

macro(run_conan conan_txt_path)
  # Conan configuration Store the value of CMAKE_PREFIX_PATH before executing the conan necessary operations. This was
  # added to warn the user if conan modifies this variable, since this modification leads to an error where catkin does
  # not place the final artifacts (libraries) in the correct destinations, leading to errors when loading the plugins
  # from gazebo
  set(CMAKE_PREFIX_PATH_BEFORE_CONAN ${CMAKE_PREFIX_PATH})

  # Set a default build type if none was specified
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
    set(CMAKE_BUILD_TYPE
        RelWithDebInfo
        CACHE STRING "Choose the type of build." FORCE
    )
    # Set the possible values of build type for cmake-gui, ccmake
    set_property(
      CACHE CMAKE_BUILD_TYPE
      PROPERTY STRINGS
               "Debug"
               "Release"
               "MinSizeRel"
               "RelWithDebInfo"
    )
  endif()
  if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(
      DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.16.1/conan.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/conan.cmake"
      EXPECTED_HASH SHA256=396e16d0f5eabdc6a14afddbcfff62a54a7ee75c6da23f32f7a31bc85db23484
      TLS_VERIFY ON
    )
  endif()
  include(${CMAKE_CURRENT_BINARY_DIR}/conan.cmake)
  # Add conan center as a remote to download the packages from
  conan_add_remote(
    NAME
    conan-center
    URL
    https://center.conan.io
  )

  conan_cmake_run(
    CONANFILE
    ${conan_txt_path} # Read the options from the conanfile.txt file.
    BUILD
    cascade # If any dependency of a package was updated, rebuild-it.
    SETTINGS
    build_type=Release
  )
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_MODULE_PATH})
  message(STATUS "Value of CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

  set(CMAKE_PREFIX_PATH_AFTER_CONAN ${CMAKE_PREFIX_PATH})
  message(STATUS "After conan:  ${CMAKE_PREFIX_PATH_AFTER_CONAN}")
  message(STATUS "Before conan: ${CMAKE_PREFIX_PATH_BEFORE_CONAN}")

  if(NOT
     "${CMAKE_PREFIX_PATH_AFTER_CONAN}"
     STREQUAL
     "${CMAKE_PREFIX_PATH_BEFORE_CONAN}"
  )
    message(WARNING "Conan modified the CMAKE_PREFIX_PATH variable from ${CMAKE_PREFIX_PATH_BEFORE_CONAN} to\
      ${CMAKE_PREFIX_PATH_AFTER_CONAN}. This can lead to an error in which catkin does not place the final compilation\
      artifacts in the correct place. Please check if the correct options where passed to conan_cmake_run."
    )
  endif()
endmacro()
