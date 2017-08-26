# - Try to find PMDSDK headers and libraries.
#
# Usage of this module as follows:
#
#     find_package(PMDSDK)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  PMDSDK_ROOT_DIR  Set this variable to the root installation of
#                   PMDSDK if the module has problems finding
#                   the proper installation path.
#
# Variables defined by this module:
#
#  PMDSDK_FOUND              System has PMDSDK libs/headers
#  PMDSDK_LIBRARIES          The PMDSDK libraries
#  PMDSDK_INCLUDE_DIR        The location of PMDSDK headers

find_path(PMDSDK_ROOT_DIR
    NAMES include/pmdsdk2.h
)

find_library(PMDSDK_LIBRARIES
    NAMES pmdaccess2
    HINTS ${PMDSDK_ROOT_DIR}/lib C:/PMDSDK/lib /usr/local/pmd/lib $ENV{PMDDIR}/lib
)

find_path(PMDSDK_INCLUDE_DIR
    NAMES pmdsdk2.h
    HINTS ${PMDSDK_ROOT_DIR}/include C:/PMDSDK/lib /usr/local/pmd/include $ENV{PMDDIR}/include
)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PMDSDK DEFAULT_MSG
    PMDSDK_LIBRARIES
    PMDSDK_INCLUDE_DIR
)
