cmake_minimum_required( VERSION 3.18.4 )
cmake_policy( VERSION 3.18.4 )


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
set( SLAM_DEMO_NAME "OpenARK_slam_demo")
set( SLAM_REPLAYING_NAME "OpenARK_slam_replaying")

option( BUILD_SLAM_DEMO "BUILD_SLAM_DEMO" ON)
option( BUILD_SLAM_REPLAYING "BUILD_SLAM_REPLAYING" ON)

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
#flag
FIND_PACKAGE(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR}) 
IF(Boost_FOUND)
  MESSAGE(STATUS "Found Boost: ${Boost_LIBRARIES}")
ENDIF(Boost_FOUND)

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

# # require Open3D
# find_package( Open3D REQUIRED )
# include_directories(${Open3D_INCLUDE_DIRS})
# link_directories(${Open3D_LIBRARY_DIRS})
# message(STATUS "Found Open3D: ${Open3D_INCLUDE_DIRS} ${Open3D_LIBRARIES}")

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
  Avatar.cpp
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
  #${INCLUDE_DIR}/SegmentedMesh.h
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

if( NOT realsense2_FOUND )
  set( _RSSDK2_ "//" )
else()
  include_directories( ${realsense2_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} RS2Camera.cpp D435iCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/RS2Camera.h ${INCLUDE_DIR}/D435iCamera.h ${INCLUDE_DIR}/D435iCalibWriter.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${realsense2_LIBRARY} )
endif( NOT realsense2_FOUND )


if ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
  set( _DEBUG_ "//" )
endif ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )

configure_file( Version.h.in ${INCLUDE_DIR}/Version.h )
set( HEADERS ${HEADERS} ${INCLUDE_DIR}/Version.h )

# download required files
if (NOT EXISTS "${PROJECT_SOURCE_DIR}/config/brisk_vocab.bn")
    message( STATUS "Downloading Brisk vocab to config/brisk_vocab.bn..." )
    file(DOWNLOAD "https://github.com/sxyu/OpenARK-Deps/releases/download/0.0.1/brisk_vocab.bn" "${PROJECT_SOURCE_DIR}/config/brisk_vocab.bn")
endif()

add_library( ${LIB_NAME} STATIC "${INCLUDE_DIR}/Core.h" ${SOURCES} ${HEADERS} )
 set_target_properties( ${LIB_NAME} PROPERTIES OUTPUT_NAME "openark")
# set_target_properties( ${LIB_NAME} PROPERTIES OUTPUT_NAME
        # "openark_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}" )

set_property( TARGET ${LIB_NAME} PROPERTY DBG_POSTFIX d )
target_link_libraries( ${LIB_NAME} ${DEPENDENCIES} )
target_include_directories( ${LIB_NAME} PRIVATE ${INCLUDE_DIR} )


# if( ${BUILD_SLAM_DEMO} AND realsense2_FOUND )
#     add_executable( ${SLAM_DEMO_NAME} SlamDemo435i.cpp )
#     target_include_directories( ${SLAM_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
#     set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES OUTPUT_NAME ${SLAM_DEMO_NAME} )
#     set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
#     if ( MSVC )
#         target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} )
#     else()
#         target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIB_NAME} GLU GL glut )
#     endif ( MSVC )
#     add_executable( d435i_intrinsics_writer D435iCalibWriter.cpp )
#     target_include_directories( d435i_intrinsics_writer PRIVATE ${INCLUDE_DIR} )
#     target_link_libraries( d435i_intrinsics_writer ${DEPENDENCIES} ${LIB_NAME} )
#     #set_target_properties( d435i_intrinsics_writer PROPERTIES OUTPUT_NAME d435i_intrinsics_writer )
#     set_target_properties( d435i_intrinsics_writer PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
# endif( ${BUILD_SLAM_DEMO} AND realsense2_FOUND )

if(${BUILD_SLAM_REPLAYING} )
    add_executable(moonwon 
        SlamReplaying.cpp 
        ${INCLUDE_DIR}/Util.h
        ${INCLUDE_DIR}/MockD435iCamera.h
        ${INCLUDE_DIR}/OkvisSLAMSystem.h
        ${INCLUDE_DIR}/glfwManager.h
        ${INCLUDE_DIR}/concurrency.h
    )
    target_link_libraries(moonwon 
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
        ${LIB_NAME}
    )
    target_include_directories( moonwon PRIVATE 
    "${PROJECT_SOURCE_DIR}/include" 
    ${OKVIS_INCLUDE_DIRS}
    ${INCLUDE_DIR} 
    )
endif(${BUILD_SLAM_REPLAYING} )


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


# Create source group for headers
source_group( "Header Files" FILES ${HEADERS} )
