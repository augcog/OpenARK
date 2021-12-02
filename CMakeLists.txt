cmake_minimum_required( VERSION 3.5 )
cmake_policy( VERSION 3.5 )
#[[ For OpenARK Ubuntu and Jetson Xavier NX, 
cmake_minimum_required( VERSION 3.18 )
cmake_policy( VERSION 3.18 )
]]

project( OpenARK )
set( OpenARK_VERSION_MAJOR 0 )
set( OpenARK_VERSION_MINOR 9 )
set( OpenARK_VERSION_PATCH 3 )  
set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" )
set( SRC_DIR "${PROJECT_SOURCE_DIR}/src")

set( CMAKE_CXX_STACK_SIZE "10000000" )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release )
endif()

message(STATUS "CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
  # NVIDIA Jetson Xavier NX 
  add_definitions(-D__ARM_NEON__)
  message(STATUS "Host System Processor is ${CMAKE_HOST_SYSTEM_PROCESSOR}, will attempt to use NEON.")
endif()

message(STATUS "CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME}")
if(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux")
    message(STATUS "Host System Name is ${CMAKE_HOST_SYSTEM_NAME}.")

    # Eigen Fix for OpenARK Ubuntu
    add_definitions(-DEIGEN_DONT_ALIGN=1)
    add_definitions(-DEIGEN_DONT_VECTORIZE=1)
    message(STATUS "EIGEN_FIX_UBUNTU is activated.")

    # Using Open3D 0.12.0 instead of 0.9.0 for Eigen Fix
    add_definitions(-D__OPEN3D_V12__)
    message(STATUS "will attempt to use Open3D 0.12.0 for Eigen Fix.")
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

# Options.
option( BUILD_HAND_DEMO "BUILD_HAND_DEMO" ON )
option( BUILD_AVATAR_DEMO "BUILD_AVATAR_DEMO" ON )
option( BUILD_SLAM_DEMO "BUILD_SLAM_DEMO" ON )
option( BUILD_3DRECON_DEMO "BUILD_3DRECON_DEMO" ON )
option( BUILD_DATA_RECORDING "BUILD_DATA_RECORDING" OFF )
option( BUILD_SLAM_RECORDING "BUILD_SLAM_RECORDING" ON )
option( BUILD_SLAM_REPLAYING "BUILD_SLAM_REPLAYING" ON )
option( BUILD_TESTS "BUILD_TESTS" OFF )
option( BUILD_UNITY_PLUGIN "BUILD_UNITY_PLUGIN" OFF )
option( USE_AZURE_KINECT_SDK "USE_AZURE_KINECT_SDK" ON )
option( USE_RSSDK2 "USE_RSSDK2" ON )
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

if ( CMAKE_COMPILER_IS_GNUCXX )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -O3 -O3 -ffast-math -funroll-all-loops -mavx -mtune=native" )
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
link_directories(${Open3D_LIBRARY_DIRS})
message(STATUS "Found Open3D: ${Open3D_INCLUDE_DIRS} ${Open3D_LIBRARIES}")

set( _FOUND_CAMERA_LIB OFF )
# azure kinect camera is now a required camera
set( k4a_FOUND OFF )
if( USE_AZURE_KINECT_SDK )
  find_package( k4a REQUIRED )
  if( k4a_FOUND)
      message( STATUS "Found k4a (Azure Kinect SDK)" )
  endif( k4a_FOUND )
  set( _FOUND_CAMERA_LIB ON )
endif()

set( realsense2_FOUND OFF )
if( USE_RSSDK2 OR NOT _FOUND_CAMERA_LIB )
  find_package( realsense2 )
  if( realsense2_FOUND )
       message( STATUS "Found librealsense2: ${realsense2_INCLUDE_DIR}" )
  endif( realsense2_FOUND )
  set( _FOUND_CAMERA_LIB ON )
endif()

set( RSSDK_FOUND OFF )
if( USE_RSSDK OR NOT _FOUND_CAMERA_LIB )
  find_package( RSSDK )
  if( RSSDK_FOUND )
       message( STATUS "Found RSSDK: ${RSSDK_INCLUDE_DIR}" )
  endif( RSSDK_FOUND )
  set( _FOUND_CAMERA_LIB ON )
endif()

set( PMDSDK_FOUND OFF )
if( USE_PMDSDK OR NOT _FOUND_CAMERA_LIB )
  find_package( PMDSDK REQUIRED )
  if( PMDSDK_FOUND )
     message( STATUS "Found PMDSDK: ${PMDSDK_INCLUDE_DIR}" )
  endif( PMDSDK_FOUND )
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
  ${Open3D_LIBRARIES}
)

add_definitions(
  ${PCL_DEFINITIONS}
)
foreach( DEFINITION ${PCL_DEFINITIONS} )
  set( TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}" )
endforeach()

set(
  SOURCES
  ${SRC_DIR}/util/FrameObject.cpp
  ${SRC_DIR}/camera/webcam/Webcam.cpp
  ${SRC_DIR}/camera/DepthCamera.cpp
  ${SRC_DIR}/camera/RGBCamera.cpp
  ${SRC_DIR}/camera/StereoCamera.cpp
  ${SRC_DIR}/camera/camera_util/Calibration.cpp
  ${SRC_DIR}/util/Util.cpp
  ${SRC_DIR}/util/Visualizer.cpp
  ${SRC_DIR}/camera/MockCamera.cpp
  ${SRC_DIR}/camera/MockD435iCamera.cpp
  ${SRC_DIR}/util/glfwManager.cpp
  ${SRC_DIR}/slam/OkvisSLAMSystem.cpp
  ${SRC_DIR}/3drecon/SaveFrame.cpp
  ${SRC_DIR}/util/SegmentedMesh.cpp
  ${SRC_DIR}/3drecon/3DReconDemo.cpp
)

set(
  HEADERS
  ${INCLUDE_DIR}/openark/util/FrameObject.h
  ${INCLUDE_DIR}/openark/camera/webcam/Webcam.h
  ${INCLUDE_DIR}/openark/camera/DepthCamera.h
  ${INCLUDE_DIR}/openark/camera/RGBCamera.h
  ${INCLUDE_DIR}/openark/camera/StereoCamera.h
  ${INCLUDE_DIR}/openark/camera/camera_util/Calibration.h
  ${INCLUDE_DIR}/openark/util/Util.h
  ${INCLUDE_DIR}/openark/util/Visualizer.h
  ${INCLUDE_DIR}/openark/camera/MockCamera.h
  ${INCLUDE_DIR}/openark/camera/MockD435iCamera.h
  ${INCLUDE_DIR}/openark/util/nanoflann.hpp
  ${INCLUDE_DIR}/openark/slam/SLAMSystem.h
  ${INCLUDE_DIR}/openark/slam/OkvisSLAMSystem.h
  ${INCLUDE_DIR}/openark/util/glfwManager.h
  ${INCLUDE_DIR}/openark/camera/CameraSetup.h
  ${INCLUDE_DIR}/openark/slam/SparseMap.h
  ${INCLUDE_DIR}/openark/slam/PointCostSolver.h
  ${INCLUDE_DIR}/openark/slam/PoseGraphSolver.h
  ${INCLUDE_DIR}/openark/util/Types.h
  ${INCLUDE_DIR}/openark/slam/CorrespondenceRansac.h
  ${INCLUDE_DIR}/openark/util/UKF.h
  ${INCLUDE_DIR}/openark/util/SaveFrame.h
  ${INCLUDE_DIR}/openark/util/SegmentedMesh.h
  ${INCLUDE_DIR}/stdafx.h
)

if (${BUILD_HAND_DEMO} OR ${BUILD_AVATAR_DEMO}) 
  set(
    SOURCES
    ${SRC_DIR}/openark/hand_and_avatar/FramePlane.cpp
    ${SRC_DIR}/openark/hand_and_avatar/StreamingAverager.cpp
    ${SRC_DIR}/openark/hand_and_avatar/Detector.cpp
    ${SRC_DIR}/openark/hand_and_avatar/PlaneDetector.cpp
  )
  set(
    HEADERS
    ${INCLUDE_DIR}/openark/hand_and_avatar/DetectionParams.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/FramePlane.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/StreamingAverager.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/Detector.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/PlaneDetector.h
  )
endif(${BUILD_HAND_DEMO} OR ${BUILD_AVATAR_DEMO})

if (${BUILD_HAND_DEMO}) 
  set(
    SOURCES
    ${SRC_DIR}/openark/hand_and_avatar/hand/Hand.cpp 
    ${SRC_DIR}/openark/hand_and_avatar/hand/HandClassifier.cpp
    ${SRC_DIR}/openark/hand_and_avatar/hand/HandDetector.cpp
  )
  set(
    HEADERS
    ${INCLUDE_DIR}/openark/hand_and_avatar/hand/Hand.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/hand/HandClassifier
    ${INCLUDE_DIR}/openark/hand_and_avatar/hand/HandDetector.h
  )
endif(${BUILD_HAND_DEMO})

if (${BUILD_AVATAR_DEMO}) 
  set(
    SOURCES
    ${SRC_DIR}/openark/hand_and_avatar/avatar/HumanDetector.cpp
    ${SRC_DIR}/openark/hand_and_avatar/avatar/HumanBody.cpp
    ${SRC_DIR}/openark/hand_and_avatar/avatar/Avatar.cpp
  )
  set(
    HEADERS
    ${INCLUDE_DIR}/openark/hand_and_avatar/avatar/HumanDetector.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/avatar/HumanBody.h
    ${INCLUDE_DIR}/openark/hand_and_avatar/avatar/Avatar.h
  )
endif(${BUILD_AVATAR_DEMO})

if(WIN32)
  add_subdirectory(third_party/glfw)
  list(APPEND DEPENDENCIES glfw3)
else()
    # Find glfw header
    find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
        PATHS /usr/X11R6/include
              /usr/include/X11
              /opt/graphics/OpenGL/include
              /opt/graphics/OpenGL/contrib/libglfw
              /usr/local/include
              /usr/include/GL
              /usr/include
    )
    # Find glfw library
    find_library(GLFW_LIBRARIES NAMES glfw glfw3
            PATHS /usr/lib64
                  /usr/lib
                  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                  /usr/local/lib64
                  /usr/local/lib
                  /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                  /usr/X11R6/lib
    )
    list(APPEND DEPENDENCIES m ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES})
    include_directories(${GLFW_INCLUDE_DIR})
endif() 

if( NOT k4a_FOUND )
  set( _AZURE_KINECT_SDK_ "//" )
else()
  set( SOURCES ${SOURCES} ${SRC_DIR}/camera/AzureKinectCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/openark/camera/AzureKinectCamera.h )
  set( DEPENDENCIES ${DEPENDENCIES} k4a::k4a )
endif( NOT k4a_FOUND )

if( NOT realsense2_FOUND )
  set( _RSSDK2_ "//" )
else()
  include_directories( ${realsense2_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} ${SRC_DIR}/camera/RS2Camera.cpp ${SRC_DIR}/camera/D435iCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/openark/camera/RS2Camera.h ${INCLUDE_DIR}/openark/camera/D435iCamera.h ${INCLUDE_DIR}/openark/camera/D435iCalibWriter.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${realsense2_LIBRARY} )
endif( NOT realsense2_FOUND )

if( NOT RSSDK_FOUND )
  set( _RSSDK_ "//" )
else()
  include_directories( ${RSSDK_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} ${SRC_DIR}/camera/SR300Camera.cpp ${SRC_DIR}/camera/camera_util/Converter.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/openark/camera/SR300Camera.h ${INCLUDE_DIR}/openark/camera/Converter.h )
endif( NOT RSSDK_FOUND )

if( NOT PMDSDK_FOUND )
  set( _PMDSDK_ "//" )
else()
  include_directories( ${PMDSDK_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} ${SRC_DIR}/camera/PMDCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/openark/camera/PMDCamera.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${PMDSDK_LIBRARIES} )
endif( NOT PMDSDK_FOUND )

if ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
  set( _DEBUG_ "//" )
endif ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )

configure_file( Version.h.in ${INCLUDE_DIR}/Version.h )
set( HEADERS ${HEADERS} ${INCLUDE_DIR}/Version.h )

# MSVC special options
if ( MSVC )
    # MSVC precompiled header
    set( PRECOMPILED_BINARY "stdafx.pch" )
    set( PRECOMPILED_HEADER "stdafx.h" )
    set( PRECOMPILED_SOURCE "stdafx.cpp" )

    set_source_files_properties( ${PRECOMPILED_SOURCE}
                                PROPERTIES COMPILE_FLAGS "/Yc\"${PRECOMPILED_HEADER}\" /Fp\"${PRECOMPILED_BINARY}\""
                                           OBJECT_OUTPUTS "${PRECOMPILED_HEADER}" )
    set_source_files_properties( ${SOURCES}
                                PROPERTIES COMPILE_FLAGS "/Yu\"${PRECOMPILED_HEADER}\" /FI\"${PRECOMPILED_HEADER}\" /Fp\"${PRECOMPILED_BINARY}\""
                                           OBJECT_DEPENDS "${PRECOMPILED_HEADER}" )
    set( SOURCES ${SOURCES} ${PRECOMPILED_SOURCE} )
    add_definitions( -D_CRT_SECURE_NO_WARNINGS )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /Ot /Z7 /Zm1010 /MP" )
    set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT /DEBUG" )
    set( CMAKE_SHARED_LINKER_FLAGS  "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT" )
    set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD" )
endif ( MSVC )

# download required files
if (NOT EXISTS "${PROJECT_SOURCE_DIR}/config/brisk_vocab.bn")
    message( STATUS "Downloading Brisk vocab to config/brisk_vocab.bn..." )
    file(DOWNLOAD "https://github.com/sxyu/OpenARK-Deps/releases/download/0.0.1/brisk_vocab.bn" "${PROJECT_SOURCE_DIR}/config/brisk_vocab.bn")
endif()

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/config/face/lbfmodel.yaml")
    message( STATUS "Downloading face landmarks model to config/face/lbfmodel.yaml..." )
    file(DOWNLOAD "https://raw.githubusercontent.com/kurnianggoro/GSOC2017/master/data/lbfmodel.yaml" "${PROJECT_SOURCE_DIR}/config/face/lbfmodel.yaml")
endif()

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/config/pose-net/pose.caffemodel")
    message( STATUS "Downloading pose net model to config/pose-net/pose.caffemodel..." )
    file(DOWNLOAD "https://github.com/sxyu/OpenARK-Deps/releases/download/0.0.1/pose.caffemodel" "${PROJECT_SOURCE_DIR}/config/pose-net/pose.caffemodel")
endif()

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/data/avatar-model/model.pcd")
    message( WARNING "SMPL model files are not present, so the avatar module will not work. To use the avatar module and/or run the avatar demo, you will need to download the SMPL model manually. Please follow the instructions in data/avatar-model/README.md. Note that SMPL is available for non-commercial, academic research purposes only." )
endif()

add_library( ${LIB_NAME} STATIC "${INCLUDE_DIR}/openark/util/Core.h" ${SOURCES} ${HEADERS} )
set_target_properties( ${LIB_NAME} PROPERTIES OUTPUT_NAME
        "openark_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}" )

set_property( TARGET ${LIB_NAME} PROPERTY DBG_POSTFIX d )
target_link_libraries( ${LIB_NAME} ${DEPENDENCIES} )
target_include_directories( ${LIB_NAME} PRIVATE ${INCLUDE_DIR} )

if ( MSVC )
    set_target_properties( ${LIB_NAME} PROPERTIES COMPILE_FLAGS "/MD" )
endif()

if( ${BUILD_HAND_DEMO} )
    add_executable( ${DEMO_NAME} ${SRC_DIR}/hand_and_avatar/hand/HandDemo.cpp )
    target_include_directories( ${DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( ${DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    set_target_properties( ${DEMO_NAME} PROPERTIES OUTPUT_NAME ${DEMO_NAME} )
    set_target_properties( ${DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_HAND_DEMO} )

if( ${BUILD_AVATAR_DEMO} )
    add_executable( ${AVATAR_DEMO_NAME} ${SRC_DIR}/hand_and_avatar/avatar/AvatarDemo.cpp )
    target_include_directories( ${AVATAR_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( ${AVATAR_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    set_target_properties( ${AVATAR_DEMO_NAME} PROPERTIES OUTPUT_NAME ${AVATAR_DEMO_NAME} )
    set_target_properties( ${AVATAR_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_AVATAR_DEMO} )

if( ${BUILD_SLAM_DEMO} AND realsense2_FOUND )
    add_executable( ${SLAM_DEMO_NAME} ${SRC_DIR}/slam/SlamDemo435i.cpp )
    target_include_directories( ${SLAM_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES OUTPUT_NAME ${SLAM_DEMO_NAME} )
    set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    if ( MSVC )
        target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    else()
        target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} GLU GL glut )
    endif ( MSVC )
    add_executable( d435i_intrinsics_writer ${SRC_DIR}/camera/D435iCalibWriter.cpp )
    target_include_directories( d435i_intrinsics_writer PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( d435i_intrinsics_writer ${DEPENDENCIES} ${LIB_NAME} )
    #set_target_properties( d435i_intrinsics_writer PROPERTIES OUTPUT_NAME d435i_intrinsics_writer )
    set_target_properties( d435i_intrinsics_writer PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_SLAM_DEMO} AND realsense2_FOUND )

if( ${BUILD_3DRECON_DEMO} AND realsense2_FOUND )
    add_executable( ${3DRECON_DATA_NAME} ${SRC_DIR}/3drecon/3DReconDemo.cpp )
    target_include_directories( ${3DRECON_DATA_NAME} PRIVATE ${INCLUDE_DIR} )
    set_target_properties( ${3DRECON_DATA_NAME} PROPERTIES OUTPUT_NAME ${3DRECON_DATA_NAME} )
    set_target_properties( ${3DRECON_DATA_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    if ( MSVC )
        target_link_libraries( ${3DRECON_DATA_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    else()
        target_link_libraries( ${3DRECON_DATA_NAME} ${DEPENDENCIES} ${LIB_NAME} GLU GL glut )
    endif ( MSVC )
endif( ${BUILD_3DRECON_DEMO} AND realsense2_FOUND )


if( ${BUILD_DATA_RECORDING} )
    add_executable( ${DATA_RECORDING_NAME} ${SRC_DIR}/hand_and_avatar/DataRecording.cpp )
    target_include_directories( ${DATA_RECORDING_NAME} PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( ${DATA_RECORDING_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    set_target_properties( ${DATA_RECORDING_NAME} PROPERTIES OUTPUT_NAME ${DATA_RECORDING_NAME} )
    set_target_properties( ${DATA_RECORDING_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_DATA_RECORDING} )

if( ${BUILD_SLAM_RECORDING} )
    add_executable( ${SLAM_RECORDING_NAME} ${SRC_DIR}/slam/SlamRecording.cpp )
    target_include_directories( ${SLAM_RECORDING_NAME} PRIVATE ${INCLUDE_DIR} )
    set_target_properties( ${SLAM_RECORDING_NAME} PROPERTIES OUTPUT_NAME ${SLAM_RECORDING_NAME} )
    set_target_properties( ${SLAM_RECORDING_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    if ( MSVC )
        target_link_libraries( ${SLAM_RECORDING_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    else()
        target_link_libraries( ${SLAM_RECORDING_NAME} ${DEPENDENCIES} ${LIB_NAME} GLU GL glut )
    endif ( MSVC )
endif( ${BUILD_SLAM_RECORDING} )

if( ${BUILD_SLAM_REPLAYING} )
    add_executable( ${SLAM_REPLAYING_NAME} ${SRC_DIR}/slam/SlamReplaying.cpp )
    target_include_directories( ${SLAM_REPLAYING_NAME} PRIVATE ${INCLUDE_DIR} )
    set_target_properties( ${SLAM_REPLAYING_NAME} PROPERTIES OUTPUT_NAME ${SLAM_REPLAYING_NAME} )
    set_target_properties( ${SLAM_REPLAYING_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    if ( MSVC )
        target_link_libraries( ${SLAM_REPLAYING_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    else()
        target_link_libraries( ${SLAM_REPLAYING_NAME} ${DEPENDENCIES} ${LIB_NAME} GLU GL glut )
    endif ( MSVC )
endif( ${BUILD_SLAM_REPLAYING} )

# Unity plugin currently only supports Windows
if( ${BUILD_UNITY_PLUGIN} AND MSVC )
    add_library( ${UNITY_PLUGIN_NAME} SHARED "unity/native/UnityInterface.cpp" "unity/native/UnityInterface.h" "unity/README.md" )
    set( HEADERS ${HEADERS} "unity/native/UnityInterface.h" )
    target_link_libraries( ${UNITY_PLUGIN_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    if ( MSVC )
        set_target_properties( ${UNITY_PLUGIN_NAME} PROPERTIES COMPILE_FLAGS "/MD" )
    endif ( MSVC )
    set_property( TARGET ${UNITY_PLUGIN_NAME} PROPERTY DBG_POSTFIX d )
    target_include_directories( ${UNITY_PLUGIN_NAME} PRIVATE ${INCLUDE_DIR} "unity/native" )
    set_target_properties( ${UNITY_PLUGIN_NAME} PROPERTIES OUTPUT_NAME "openark_unity_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}_native" )
endif( ${BUILD_UNITY_PLUGIN} AND MSVC )

if( ${BUILD_TESTS} )
  if( NOT EXISTS ${PROJECT_SOURCE_DIR}/OpenARK_test )
    execute_process(
      COMMAND git clone https://github.com/augcog/OpenARK_test.git
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} )
  endif( NOT EXISTS ${PROJECT_SOURCE_DIR}/OpenARK_test )

  add_executable( ${TEST_NAME} "OpenARK_test/test.cpp" "OpenARK_test/TestCamera.cpp" "OpenARK_test/TestCamera.h" )
  set( HEADERS ${HEADERS} "OpenARK_test/TestCamera.h" )

  target_link_libraries( ${TEST_NAME} ${DEPENDENCIES} ${LIB_NAME} )
  set_target_properties( ${TEST_NAME} PROPERTIES
    COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )

  target_include_directories( ${TEST_NAME} PRIVATE ${INCLUDE_DIR} )
endif( ${BUILD_TESTS} )

# Create source group for headers
source_group( "Header Files" FILES ${HEADERS} )
