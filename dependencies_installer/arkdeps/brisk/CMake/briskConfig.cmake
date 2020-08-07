# - Config file for the OKVIS package
# It defines the following variables
#  BRISK_INCLUDE_DIRS - include directories for FooBar
#  BRISK_LIBRARIES    - libraries to link against
#  BRISK_EXECUTABLE   - the okvis_app_synchronous executable
 
# Compute paths
get_filename_component(BRISK_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(BRISK_ROOT_DIR "${BRISK_CMAKE_DIR}" PATH)
set(BRISK_INCLUDE_DIRS "${BRISK_ROOT_DIR}/include")

message(${BRISK_ROOT_DIR})
message(${BRISK_CMAKE_DIR})

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET brisk AND NOT BRISK_BINARY_DIR)
  include("${BRISK_CMAKE_DIR}/briskTargets.cmake")
endif()


 
# These are IMPORTED targets created by okvisTargets.cmake
set(BRISK_LIBRARIES 
    "${BRISK_ROOT_DIR}/lib/brisk.lib"
    "${BRISK_ROOT_DIR}/lib/agast.lib"
)
#set(BRISK_EXECUTABLE demo)
