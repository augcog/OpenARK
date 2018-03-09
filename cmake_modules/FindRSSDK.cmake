# - Try to find Windows RSSDK headers and libraries.
#
# Usage of this module as follows:
#
#     find_package(RSSDK)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  RSSDK_DIR  Set this variable to the root installation of
#                   RSSDK if the module has problems finding
#                   the proper installation path.
#
# Variables defined by this module:
#
#  RSSDK_FOUND              RSSDK_DIR is set correctly
#
# All that is needed to use RSSDK is that the environment variable 
# RSSDK_DIR is set to the directory of the SDK
# This file simply checks that the environment variable is set correctly
#

find_path(RSSDK_DIR
    NAMES include/pxcbase.h
)

find_path(RSSDK_INCLUDE_DIR
    NAMES pxcbase.h
    HINTS $ENV{RSSDK_DIR}/include 
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RSSDK DEFAULT_MSG
    RSSDK_DIR RSSDK_INCLUDE_DIR
)
