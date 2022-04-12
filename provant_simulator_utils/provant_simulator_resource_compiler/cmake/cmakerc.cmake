# This file is part of the ProVANT simulator project. Licensed under the terms of the MIT open source license. More
# details at https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

macro(add_cmakerc)

  file(DOWNLOAD "https://raw.githubusercontent.com/vector-of-bool/cmrc/master/CMakeRC.cmake"
       "${CMAKE_BINARY_DIR}/CMakeRC.cmake"
  )
  include("${CMAKE_BINARY_DIR}/CMakeRC.cmake")

endmacro()
