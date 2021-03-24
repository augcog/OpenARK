cmake_minimum_required( VERSION 3.5 )
cmake_policy( VERSION 3.5 )


project( OpenARK )
set( OpenARK_VERSION_MAJOR 0 )
set( OpenARK_VERSION_MINOR 9 )
set( OpenARK_VERSION_PATCH 3 )
set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" )

set( CMAKE_CXX_STACK_SIZE "10000000" )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release )
endif()

set( LIB_NAME "OpenARK" )
set( DEMO_NAME "OpenARK_hand_demo" )
set( AVATAR_DEMO_NAME "OpenARK_avatar_demo")
set( SLAM_DEMO_NAME "OpenARK_SLAM_demo")
set( 3DRECON_DATA_NAME "3dRecon_Data_Recording")
set( DATA_RECORDING_NAME "OpenARK_data_recording")
set( SLAM_RECORDING_NAME "OpenARK_slam_recording")
set( SLAM_REPLAYING_NAME "OpenARK_slam_replaying")
set( TEST_NAME "OpenARK_test" )
set( UNITY_PLUGIN_NAME "UnityPlugin" )

option( BUILD_HAND_DEMO "BUILD_HAND_DEMO" OFF )
option( BUILD_AVATAR_DEMO "BUILD_AVATAR_DEMO" OFF)
option( BUILD_SLAM_DEMO "BUILD_SLAM_DEMO" OFF)
option( BUILD_3DRECON_DEMO "BUILD_3DRECON_DEMO" OFF)
option( BUILD_DATA_RECORDING "BUILD_DATA_RECORDING" OFF)
option( BUILD_SLAM_RECORDING "BUILD_SLAM_RECORDING" OFF)
option( BUILD_SLAM_REPLAYING "BUILD_SLAM_REPLAYING" ON)
option( BUILD_TESTS "BUILD_TESTS" OFF )
option( BUILD_UNITY_PLUGIN "BUILD_UNITY_PLUGIN" OFF )
option( USE_AZURE_KINECT_SDK "USE_AZURE_KINECT_SDK" OFF )
option( USE_RSSDK2 "USE_RSSDK2" OFF )
option( USE_RSSDK "USE_RSSDK" OFF )
option( USE_PMDSDK "USE_PMDSDK" OFF )

include( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
CHECK_CXX_COMPILER_FLAG( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )

if( COMPILER_SUPPORTS_CXX11 )
    message(STATUS "c++11 supported")
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
elseif( COMPILER_SUPPORTS_CXX0X )
    message(status "c++ something supported")
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
else()
    message( STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
endif()

message(STATUS "using c++17")
add_definitions(-DEIGEN_DONT_ALIGN=1)
add_definitions(-DEIGEN_DONT_VECTORIZE=1)


if ( CMAKE_COMPILER_IS_GNUCXX )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -O3 -g" )
endif ( CMAKE_COMPILER_IS_GNUCXX )

if( DEFINED ENV{ARK_DEPS_DIR} )
    message( STATUS "Detected ARK_DEPS_DIR: $ENV{ARK_DEPS_DIR}" )
    include( $ENV{ARK_DEPS_DIR}/CMakeLists.txt )
endif( DEFINED ENV{ARK_DEPS_DIR} )

set( Boost_USE_STATIC_LIBS ON )
set( Boost_USE_STATIC ON )

# require Eigen
if( NOT DEFINED EIGEN_INCLUDE_DIRS )
    if ( DEFINED Eigen_DIR )
        set( EIGEN_INCLUDE_DIRS ${Eigen_DIR} )
    endif ( DEFINED Eigen_DIR )
endif( NOT DEFINED EIGEN_INCLUDE_DIRS )
if ( NOT DEFINED Eigen_DIR )
    find_package( Eigen QUIET )
endif ( NOT DEFINED Eigen_DIR )

if( DEFINED EIGEN_INCLUDE_DIRS )
    message(STATUS "Using user-provided Eigen")
else()
    # use submodule automatically if not found
    message(STATUS "Using Eigen submodule, updating...")
    execute_process(COMMAND git submodule update --init --recursive
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules manually")
    else()
        message(STATUS "Submodule up to date")
    endif(NOT GIT_SUBMOD_RESULT EQUAL "0")
    set( EIGEN_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/third_party/eigen3" )
endif( DEFINED EIGEN_INCLUDE_DIRS )
set( EIGEN_INCLUDE_DIR ${EIGEN_INCLUDE_DIRS} )
include_directories(${EIGEN_INCLUDE_DIRS})
message(STATUS "Found Eigen: ${EIGEN_INCLUDE_DIRS}")

# require PCL
find_package( PCL REQUIRED )
if( PCL_FOUND )
   message( STATUS "Found PCL: ${PCL_INCLUDE_DIRS}" )
endif( PCL_FOUND )

# require OpenCV
find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
   message( STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS}" )
endif( OpenCV_FOUND )

# require glog
if( DEFINED ENV{ARK_DEPS_DIR} )
    find_package( Glog REQUIRED NO_MODULE )
else()
    find_package( Glog REQUIRED )
endif( DEFINED ENV{ARK_DEPS_DIR} )
message(STATUS "Found Glog: ${GLOG_INCLUDE_DIRS} ${GLOG_LIBRARIES}")
# fix glog windows conflicts and bugs
if(WIN32)
    add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES -DGOOGLE_GLOG_DLL_DECL= -D_MBCS)
endif(WIN32)
include_directories(${GLOG_INCLUDE_DIRS})

# require DBoW2
find_package( DBoW2 REQUIRED )
IF( DBoW2_FOUND )
    MESSAGE( STATUS "Found DBoW2: ${DBoW2_INCLUDE_DIRS} ${DBoW2_LIBRARIES}" )
   include_directories(${DBoW2_INCLUDE_DIRS})
ENDIF( DBoW2_FOUND )

# require DLoopDetector
set(ENV{DLoopDetector_INCLUDE_DIRS} "/usr/local/include/DLoopDetector")
find_package( DLoopDetector REQUIRED )
IF( DLoopDetector_FOUND )
    MESSAGE( STATUS "Found DLoopDetector: $ENV{DLoopDetector_INCLUDE_DIRS} ${DLoopDetector_LIBRARIES} ${DLoopDetector_INCLUDE_DIRS}" )
ENDIF( DLoopDetector_FOUND )

if( DEFINED ENV{ARK_DEPS_DIR} )
  include_directories(${DLoopDetector_INCLUDE_DIRS})
else()
  include_directories($ENV{DLoopDetector_INCLUDE_DIRS})
endif( DEFINED ENV{ARK_DEPS_DIR} )


# require Okvis
find_package( okvis REQUIRED )
include_directories(${OKVIS_INCLUDE_DIRS})
message(STATUS "Found Okvis: ${OKVIS_INCLUDE_DIRS} ${OKVIS_LIBRARIES}")

# require opengv (should come with Okvis)
if( DEFINED ENV{ARK_DEPS_DIR} )
  find_package( opengv REQUIRED NO_MODULE )
else()
    find_package( opengv REQUIRED )
endif( DEFINED ENV{ARK_DEPS_DIR} )
IF(OPENGV_FOUND)
    MESSAGE(STATUS "Found opengv: ${OPENGV_INCLUDE_DIR} ${OPENGV_LIBRARIES}")
ENDIF(OPENGV_FOUND)

# require Ceres
find_package( Ceres REQUIRED )
IF(Ceres_FOUND)
    MESSAGE(STATUS "Found Ceres: ${CERES_INCLUDE_DIRS} ${CERES_LIBRARIES}")
ENDIF(Ceres_FOUND)
include_directories(${CERES_INCLUDE_DIRS})

# require SuiteSparse
find_package( SuiteSparse QUIET )
IF(SuiteSparse_FOUND)
    MESSAGE(STATUS "Found SuiteSparse: ${SuiteSparse_INCLUDE_DIRS} ${SuiteSparse_LIBRARIES}")
ENDIF(SuiteSparse_FOUND)
include_directories(${SuiteSparse_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
   message(STATUS "OpenMP found")
   add_definitions(-DUSE_OPENMP)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# require Brisk
find_package( brisk REQUIRED )
include_directories(${BRISK_INCLUDE_DIRS})
message(STATUS "Found Brisk: ${BRISK_INCLUDE_DIRS} ${BRISK_LIBRARIES}")

# require Open3D
find_package( Open3D REQUIRED )
include_directories(${Open3D_INCLUDE_DIRS})
#link_directories(${Open3D_LIBRARY_DIRS})
message(STATUS "Found Open3D: ${Open3D_INCLUDE_DIRS} ${Open3D_LIBRARIES}")


set( realsense2_FOUND OFF )
if( USE_RSSDK2 OR NOT _FOUND_CAMERA_LIB )
  find_package( realsense2 )
  if( realsense2_FOUND )
       message( STATUS "Found librealsense2: ${realsense2_INCLUDE_DIR}" )
  endif( realsense2_FOUND )
  set( _FOUND_CAMERA_LIB ON )
endif()


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

set(
  DEPENDENCIES
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES} #${CXSPARSE_LIBRARIES}
  ${SuiteSparse_LIBRARIES}
  ${OKVIS_LIBRARIES}
  ${CERES_LIBRARIES}
  ${BRISK_LIBRARIES}
  ${DBoW2_LIBRARIES}
  ${OPENGV_LIBRARIES}
  ${Boost_LIBRARIES}
  #${Open3D_LIBRARIES}
)

add_definitions(
  ${PCL_DEFINITIONS}
)
foreach( DEFINITION ${PCL_DEFINITIONS} )
  set( TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}" )
endforeach()

set(
  SOURCES
  FrameObject.cpp
  Hand.cpp FramePlane.cpp
  Webcam.cpp
  DepthCamera.cpp
  RGBCamera.cpp
  StereoCamera.cpp
  StreamingAverager.cpp
  Calibration.cpp
  Util.cpp
  Visualizer.cpp
  HandClassifier.cpp
  Detector.cpp
  HandDetector.cpp
  PlaneDetector.cpp
  MockCamera.cpp
  MockD435iCamera.cpp
  HumanDetector.cpp
  HumanBody.cpp
  #Avatar.cpp
  glfwManager.cpp
  OkvisSLAMSystem.cpp
  SaveFrame.cpp
  #SegmentedMesh.cpp
)

set(
  HEADERS
  ${INCLUDE_DIR}/FrameObject.h
  ${INCLUDE_DIR}/DetectionParams.h
  ${INCLUDE_DIR}/Hand.h
  ${INCLUDE_DIR}/FramePlane.h
  ${INCLUDE_DIR}/Webcam.h
  ${INCLUDE_DIR}/DepthCamera.h
  ${INCLUDE_DIR}/RGBCamera.h
  ${INCLUDE_DIR}/StereoCamera.h
  ${INCLUDE_DIR}/StreamingAverager.h
  ${INCLUDE_DIR}/Calibration.h
  ${INCLUDE_DIR}/Util.h
  ${INCLUDE_DIR}/Visualizer.h
  ${INCLUDE_DIR}/HandClassifier.h
  ${INCLUDE_DIR}/Detector.h
  ${INCLUDE_DIR}/HandDetector.h
  ${INCLUDE_DIR}/PlaneDetector.h
  ${INCLUDE_DIR}/MockCamera.h
  ${INCLUDE_DIR}/MockD435iCamera.h
  ${INCLUDE_DIR}/HumanDetector.h
  ${INCLUDE_DIR}/HumanBody.h
  #${INCLUDE_DIR}/Avatar.h
  ${INCLUDE_DIR}/nanoflann.hpp
  ${INCLUDE_DIR}/SLAMSystem.h
  ${INCLUDE_DIR}/OkvisSLAMSystem.h
  ${INCLUDE_DIR}/glfwManager.h
  ${INCLUDE_DIR}/CameraSetup.h
  ${INCLUDE_DIR}/SparseMap.h
  ${INCLUDE_DIR}/PointCostSolver.h
  ${INCLUDE_DIR}/PoseGraphSolver.h
  ${INCLUDE_DIR}/Types.h
  ${INCLUDE_DIR}/CorrespondenceRansac.h
  ${INCLUDE_DIR}/UKF.h
  ${INCLUDE_DIR}/SaveFrame.h
  #${INCLUDE_DIR}/SegmentedMesh.h
  stdafx.h
)


if ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
  set( _DEBUG_ "//" )
endif ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )

configure_file( Version.h.in ${INCLUDE_DIR}/Version.h )
set( HEADERS ${HEADERS} ${INCLUDE_DIR}/Version.h )


# build the SegmentedMesh library
add_library( SegmentedMesh STATIC
    SegmentedMesh.cpp
    include/SegmentedMesh.h
)

# and link it                     
target_link_libraries(SegmentedMesh 
    #${Open3D_LIBRARY_DIRS}
    ${Open3D_LIBRARIES}
)
target_include_directories( SegmentedMesh PRIVATE ${Open3D_INCLUDE_DIRS} )


# Making OpenARK Library
add_library( ${LIB_NAME} STATIC "${INCLUDE_DIR}/Core.h" ${SOURCES} ${HEADERS})
set_target_properties( ${LIB_NAME} PROPERTIES OUTPUT_NAME
        "openark_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}" )

set_property( TARGET ${LIB_NAME} PROPERTY DBG_POSTFIX d )
target_link_libraries( ${LIB_NAME} ${DEPENDENCIES} SegmentedMesh)
target_include_directories( ${LIB_NAME} PRIVATE ${INCLUDE_DIR} )

# Making slam_replaying
if( ${BUILD_SLAM_REPLAYING} )
    add_executable( ${SLAM_REPLAYING_NAME} SlamReplaying.cpp )
    target_include_directories( ${SLAM_REPLAYING_NAME} PRIVATE ${INCLUDE_DIR} )
    set_target_properties( ${SLAM_REPLAYING_NAME} PROPERTIES OUTPUT_NAME ${SLAM_REPLAYING_NAME} )
    set_target_properties( ${SLAM_REPLAYING_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    if ( MSVC )
        target_link_libraries( ${SLAM_REPLAYING_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    else()
        target_link_libraries( ${SLAM_REPLAYING_NAME} ${DEPENDENCIES} ${LIB_NAME}  GLU GL glut )
    endif ( MSVC )
endif( ${BUILD_SLAM_REPLAYING} )

# Create source group for headers
source_group( "Header Files" FILES ${HEADERS} )