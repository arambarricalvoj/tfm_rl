#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "irobot_create_toolbox::irobot_create_toolbox" for configuration ""
set_property(TARGET irobot_create_toolbox::irobot_create_toolbox APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(irobot_create_toolbox::irobot_create_toolbox PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libirobot_create_toolbox.so"
  IMPORTED_SONAME_NOCONFIG "libirobot_create_toolbox.so"
  )

list(APPEND _cmake_import_check_targets irobot_create_toolbox::irobot_create_toolbox )
list(APPEND _cmake_import_check_files_for_irobot_create_toolbox::irobot_create_toolbox "${_IMPORT_PREFIX}/lib/libirobot_create_toolbox.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
