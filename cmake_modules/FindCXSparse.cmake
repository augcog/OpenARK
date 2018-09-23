# - Find CXSparse library
# Find the native CXSparse includes and library
# This module defines
#  CXSPARSE_INCLUDE_DIRS, where to find ceres.h, Set when
#                      CXSPARSE_INCLUDE_DIR is found.
#  CXSPARSE_LIBRARIES, libraries to link against to use CXSparse.
#  CXSPARSE_ROOT_DIR, The base directory to search for CXSparse.
#                  This can also be an environment variable.
#  CXSPARSE_FOUND, If false, do not try to use CXSparse.
#
# also defined, but not for general use are
#  CXSPARSE_LIBRARY, where to find the CXSparse library.

# If CXSPARSE_ROOT_DIR was defined in the environment, use it.
IF(NOT CXSPARSE_ROOT_DIR AND NOT $ENV{CXSPARSE_ROOT_DIR} STREQUAL "")
  SET(CXSPARSE_ROOT_DIR $ENV{CXSPARSE_ROOT_DIR})
ENDIF()

SET(_ceres_SEARCH_DIRS
  ${CXSPARSE_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/cxsparse
)

SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")

FIND_LIBRARY(CXSPARSE_LIBRARY
  NAMES
    cxsparse
  HINTS
    ${ARK_DEPENDENCY_DIR}/lib ${_cxsparse_SEARCH_DIRS}
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set CXSPARSE_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CXSparse DEFAULT_MSG
    CXSPARSE_LIBRARY)

IF(CXSPARSE_FOUND)
  SET(CXSPARSE_LIBRARIES ${CXSPARSE_LIBRARY})
  set(CXSPARSE_VERSION 1.11.0)
ENDIF(CXSPARSE_FOUND)

MARK_AS_ADVANCED(
  CXSPARSE_LIBRARY
)
