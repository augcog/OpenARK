# - Try to find Intel RealSense (R) SDK 2 headers and libraries.
# - Intel RealSense SDK 2: https://github.com/IntelRealSense/librealsense
#
# Usage of this module as follows:
#
#     find_package(RSSDK2)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  RSSDK2_DIR  Set this variable to the root installation of
#                   RSSDK2 if the module has problems finding
#                   the proper installation path.
#  RSSDK2_LIBRARY_DIR  Set this variable to where the RSSDK library is located
#
# Variables defined by this module:
#
#  RSSDK2_FOUND              RSSDK2_LIBRARY is set correctly
#
# All that is needed to use RSSDK2 is that the environment variable 
# RSSDK2_LIBRARY is set to the path to the RSSDK2 library
# RSSDK2_INCLUDE_DIR is set to
# This file simply checks that the environment variable is set correctly
#

find_path(RSSDK2_INCLUDE_DIR
    NAMES librealsense2/rs.hpp
    HINTS $ENV{RSSDK2_DIR}/include
)

find_library(RSSDK2_LIBRARY
    NAMES realsense2
    HINTS $ENV{RSSDK2_LIBRARY_DIR}
    PATH_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RSSDK2 DEFAULT_MSG
    RSSDK2_INCLUDE_DIR RSSDK2_LIBRARY
)
