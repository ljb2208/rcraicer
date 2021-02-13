# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/track_builder_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/track_builder_autogen.dir/ParseCache.txt"
  "track_builder_autogen"
  )
endif()
