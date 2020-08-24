# include() this file in project's CMakeLists to find all OpenARK dependencies

get_filename_component(DEPS_ROOT "${CMAKE_CURRENT_LIST_FILE}" PATH)

# set all dependency paths
set( OpenCV_DIR "${DEPS_ROOT}/opencv343-contrib" ) 
set( Eigen_DIR "${DEPS_ROOT}/eigen3" )
set( EIGEN_INCLUDE_DIRS ${Eigen_DIR} )
set( Ceres_DIR "${DEPS_ROOT}/Ceres/CMake" )
set( Glog_DIR "${DEPS_ROOT}/glog/lib/cmake/glog" ) 
set( SuiteSparse_DIR "${DEPS_ROOT}/suitesparse/lib/cmake/suitesparse-5.1.0" ) 
set( brisk_DIR "${DEPS_ROOT}/brisk/CMake" ) 
set( DBoW2_DIR "${DEPS_ROOT}/DBoW2/include/DBoW2" ) 
set( DLoopDetector_DIR "${DEPS_ROOT}/DLoopDetector/include/DLoopDetector" ) 
set( OpenGV_DIR "${DEPS_ROOT}/opengv/CMake" ) 
set( okvis_DIR "${DEPS_ROOT}/okvis/CMake" ) 
set( realsense2_DIR "${DEPS_ROOT}/librealsense2/lib/cmake/realsense2" ) 
set( Open3D_DIR "${DEPS_ROOT}/Open3D/CMake" )
