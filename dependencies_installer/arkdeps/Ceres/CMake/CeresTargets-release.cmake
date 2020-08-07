#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ceres" for configuration "Release"
set_property(TARGET ceres APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ceres PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "glog::glog;C:/Program Files/OpenARK-Deps/suitesparse/lib/metis.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libamd.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libcamd.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libccolamd.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libcolamd.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libcholmod.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libspqr.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libldl.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libbtf.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libklu.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libcxsparse.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/libumfpack.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib/suitesparseconfig.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib64/lapack_blas_windows/libblas.lib;C:/Program Files/OpenARK-Deps/suitesparse/lib64/lapack_blas_windows/liblapack.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/ceres.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS ceres )
list(APPEND _IMPORT_CHECK_FILES_FOR_ceres "${_IMPORT_PREFIX}/lib/ceres.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
