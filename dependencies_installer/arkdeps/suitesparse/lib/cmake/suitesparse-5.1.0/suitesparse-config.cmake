# Compute locations from <prefix>/@{LIBRARY_DIR@/cmake/suitesparse-<v>/<self>.cmake
get_filename_component(_SuiteSparse_SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_SuiteSparse_PREFIX "${_SuiteSparse_SELF_DIR}" PATH)
get_filename_component(_SuiteSparse_PREFIX "${_SuiteSparse_PREFIX}" PATH)
get_filename_component(_SuiteSparse_PREFIX "${_SuiteSparse_PREFIX}" PATH)

# Load the LAPACK package with which we were built.
#find_package(LAPACK CONFIG)

# Load targets from the install tree.
include(${_SuiteSparse_SELF_DIR}/SuiteSparse-targets.cmake)

# Report SuiteSparse header search locations.
set(SuiteSparse_INCLUDE_DIRS ${_SuiteSparse_PREFIX}/include)

# Report SuiteSparse libraries.
set(SuiteSparse_LIBRARIES
	# SuiteSparse::suitesparseconfig
	# SuiteSparse::amd
	# SuiteSparse::btf
	# SuiteSparse::camd
	# SuiteSparse::ccolamd
	# SuiteSparse::colamd
	# SuiteSparse::cholmod
	# SuiteSparse::cxsparse
	# SuiteSparse::klu
	# SuiteSparse::ldl
	# SuiteSparse::umfpack
	# SuiteSparse::spqr
	# SuiteSparse::metis
    "${_SuiteSparse_PREFIX}/lib/suitesparseconfig.lib"
    "${_SuiteSparse_PREFIX}/lib/libamd.lib"
    "${_SuiteSparse_PREFIX}/lib/libbtf.lib"
    "${_SuiteSparse_PREFIX}/lib/libcamd.lib"
    "${_SuiteSparse_PREFIX}/lib/libccolamd.lib"
    "${_SuiteSparse_PREFIX}/lib/libcolamd.lib"
    "${_SuiteSparse_PREFIX}/lib/libcholmod.lib"
    "${_SuiteSparse_PREFIX}/lib/libcxsparse.lib"
    "${_SuiteSparse_PREFIX}/lib/libklu.lib"
    "${_SuiteSparse_PREFIX}/lib/libldl.lib"
    "${_SuiteSparse_PREFIX}/lib/libumfpack.lib"
    "${_SuiteSparse_PREFIX}/lib/libspqr.lib"
    "${_SuiteSparse_PREFIX}/lib/metis.lib"
    "${_SuiteSparse_PREFIX}/lib64/lapack_blas_windows/libblas.lib"
    "${_SuiteSparse_PREFIX}/lib64/lapack_blas_windows/liblapack.lib"
)

unset(_SuiteSparse_PREFIX)
unset(_SuiteSparse_SELF_DIR)
