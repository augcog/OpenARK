#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "realsense2-gl::realsense2-gl" for configuration "Release"
set_property(TARGET realsense2-gl::realsense2-gl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2-gl::realsense2-gl PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/realsense2-gl.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "realsense2::realsense2"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/realsense2-gl.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2-gl::realsense2-gl )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2-gl::realsense2-gl "${_IMPORT_PREFIX}/lib/realsense2-gl.lib" "${_IMPORT_PREFIX}/bin/realsense2-gl.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
