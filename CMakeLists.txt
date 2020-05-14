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
option( BUILD_SLAM_DEMO "BUILD_SLAM_DEMO" ON)
option( BUILD_3DRECON_DEMO "BUILD_3DRECON_DEMO" ON)
option( BUILD_DATA_RECORDING "BUILD_DATA_RECORDING" OFF)
option( BUILD_SLAM_RECORDING "BUILD_SLAM_RECORDING" ON)
option( BUILD_SLAM_REPLAYING "BUILD_SLAM_REPLAYING" ON)
option( BUILD_3DRECON_DEMO "BUILD_3DRECON_DEMO" ON)
option( BUILD_TESTS "BUILD_TESTS" OFF )
option( BUILD_UNITY_PLUGIN "BUILD_UNITY_PLUGIN" ON )
option( USE_AZURE_KINECT_SDK "USE_AZURE_KINECT_SDK" OFF )
option( USE_RSSDK2 "USE_RSSDK2" ON )
option( USE_RSSDK "USE_RSSDK" OFF )
option( USE_PMDSDK "USE_PMDSDK" OFF )

include( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
CHECK_CXX_COMPILER_FLAG( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )

if( COMPILER_SUPPORTS_CXX11 )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
elseif( COMPILER_SUPPORTS_CXX0X )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
else()
    message( STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
endif()

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
set( EIGEN_INCLUDE_DIR EIGEN_INCLUDE_DIRS )
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
#if( DEFINED ENV{ARK_DEPS_DIR} )
#    find_package( Glog REQUIRED NO_MODULE )
#else()
#    find_package( Glog REQUIRED )
#endif( DEFINED ENV{ARK_DEPS_DIR} )

set( GLOG_DIR "C:/Program Files/glog/lib/cmake/glog" ) 
find_package(Glog REQUIRED)

if (Glog_FOUND)
  message( STATUS "Found GLOG" )
else()
  message( STATUS "DIDNT FIND GLOG")
endif()


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
#find_package( DLoopDetector REQUIRED )
#IF( DLoopDetector_FOUND )
#    MESSAGE( STATUS "Found DLoopDetector: ${DLoopDetector_INCLUDE_DIRS} ${DLoopDetector_LIBRARIES}" )
#   include_directories(${DLoopDetector_INCLUDE_DIRS})
#ENDIF( DLoopDetector_FOUND )

set ( DLoopDetector_INCLUDE_DIRS "C:/Program Files/DLoopDetector/include/DLoopDetector")
include_directories(${DLoopDetector_INCLUDE_DIRS})
MESSAGE( STATUS "Found DLoopDetector: ${DLoopDetector_INCLUDE_DIRS} ${DLoopDetector_LIBRARIES}" )

set( Okvis_DIR "C:/Program Files/okvis/CMake" ) 


# require Okvis
find_package( Okvis REQUIRED )
include_directories(${OKVIS_INCLUDE_DIRS})
message(STATUS "Found Okvis: ${OKVIS_INCLUDE_DIRS} ${OKVIS_LIBRARIES}")

# require OpenGV (should come with Okvis)
if( DEFINED ENV{ARK_DEPS_DIR} )
   find_package( OpenGV REQUIRED NO_MODULE )
else()
    find_package( OpenGV REQUIRED )
endif( DEFINED ENV{ARK_DEPS_DIR} )
IF(OPENGV_FOUND)
    MESSAGE(STATUS "Found OpenGV: ${OPENGV_INCLUDE_DIR} ${OPENGV_LIBRARIES}")
ENDIF(OPENGV_FOUND)

set( Ceres_DIR "C:/Program Files/Ceres/CMake" ) 
set( EIGEN_INCLUDE_DIR "C:/Program Files/OpenARK-Deps/eigen3" )

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
set( k4a_FOUND OFF )
if( USE_AZURE_KINECT_SDK )
  find_package( k4a )
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
  Avatar.cpp
  glfwManager.cpp
  OkvisSLAMSystem.cpp
  SaveFrame.cpp
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
  ${INCLUDE_DIR}/Avatar.h
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
  stdafx.h
)

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
  set( SOURCES ${SOURCES} AzureKinectCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/AzureKinectCamera.h )
  set( DEPENDENCIES ${DEPENDENCIES} k4a::k4a )
endif( NOT k4a_FOUND )

if( NOT realsense2_FOUND )
  set( _RSSDK2_ "//" )
else()
  include_directories( ${realsense2_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} RS2Camera.cpp D435iCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/RS2Camera.h ${INCLUDE_DIR}/D435iCamera.h ${INCLUDE_DIR}/D435iCalibWriter.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${realsense2_LIBRARY} )
endif( NOT realsense2_FOUND )

if( NOT RSSDK_FOUND )
  set( _RSSDK_ "//" )
else()
  include_directories( ${RSSDK_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} SR300Camera.cpp Converter.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/SR300Camera.h ${INCLUDE_DIR}/Converter.h )
endif( NOT RSSDK_FOUND )

if( NOT PMDSDK_FOUND )
  set( _PMDSDK_ "//" )
else()
  include_directories( ${PMDSDK_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} PMDCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/PMDCamera.h )
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
    set( PRECOMPILED_BINARY "$(IntDir)/stdafx.pch" )
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

add_library( ${LIB_NAME} STATIC "${INCLUDE_DIR}/Core.h" ${SOURCES} ${HEADERS} )
set_target_properties( ${LIB_NAME} PROPERTIES OUTPUT_NAME
        "openark_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}" )

set_property( TARGET ${LIB_NAME} PROPERTY DBG_POSTFIX d )
target_link_libraries( ${LIB_NAME} ${DEPENDENCIES} )
target_include_directories( ${LIB_NAME} PRIVATE ${INCLUDE_DIR} )

if ( MSVC )
    set_target_properties( ${LIB_NAME} PROPERTIES COMPILE_FLAGS "/MD" )
endif()

if( ${BUILD_HAND_DEMO} )
    add_executable( ${DEMO_NAME} HandDemo.cpp )
    target_include_directories( ${DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( ${DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    set_target_properties( ${DEMO_NAME} PROPERTIES OUTPUT_NAME ${DEMO_NAME} )
    set_target_properties( ${DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_HAND_DEMO} )

if( ${BUILD_AVATAR_DEMO} )
    add_executable( ${AVATAR_DEMO_NAME} AvatarDemo.cpp )
    target_include_directories( ${AVATAR_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( ${AVATAR_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    set_target_properties( ${AVATAR_DEMO_NAME} PROPERTIES OUTPUT_NAME ${AVATAR_DEMO_NAME} )
    set_target_properties( ${AVATAR_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_AVATAR_DEMO} )

if( ${BUILD_SLAM_DEMO} AND realsense2_FOUND )
    add_executable( ${SLAM_DEMO_NAME} SlamDemo435i.cpp )
    target_include_directories( ${SLAM_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES OUTPUT_NAME ${SLAM_DEMO_NAME} )
    set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    if ( MSVC )
        target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    else()
        target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} GLU GL glut )
    endif ( MSVC )
    add_executable( d435i_intrinsics_writer D435iCalibWriter.cpp )
    target_include_directories( d435i_intrinsics_writer PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( d435i_intrinsics_writer ${DEPENDENCIES} ${LIB_NAME} )
    #set_target_properties( d435i_intrinsics_writer PROPERTIES OUTPUT_NAME d435i_intrinsics_writer )
	set_target_properties( d435i_intrinsics_writer PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_SLAM_DEMO} AND realsense2_FOUND )

if( ${BUILD_3DRECON_DEMO} AND realsense2_FOUND )
    add_executable( ${3DRECON_DATA_NAME} 3DReconDemo.cpp )
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
    add_executable( ${DATA_RECORDING_NAME} DataRecording.cpp )
    target_include_directories( ${DATA_RECORDING_NAME} PRIVATE ${INCLUDE_DIR} )
    target_link_libraries( ${DATA_RECORDING_NAME} ${DEPENDENCIES} ${LIB_NAME} )
    set_target_properties( ${DATA_RECORDING_NAME} PROPERTIES OUTPUT_NAME ${DATA_RECORDING_NAME} )
    set_target_properties( ${DATA_RECORDING_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
endif( ${BUILD_DATA_RECORDING} )

if( ${BUILD_SLAM_RECORDING} )
    add_executable( ${SLAM_RECORDING_NAME} SlamRecording.cpp )
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
    add_executable( ${SLAM_REPLAYING_NAME} SlamReplaying.cpp )
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
