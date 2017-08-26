cmake_minimum_required(VERSION 2.8)
project( OpenARK )
set(CMAKE_CXX_STACK_SIZE "10000000")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package( PCL REQUIRED )

find_package( OpenCV REQUIRED )
if(OpenCV_FOUND)
   message(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS}")
endif(OpenCV_FOUND)

find_package( PMDSDK REQUIRED )
if(PMDSDK_FOUND)
   message(STATUS "Found OpenCV: ${PMDSDK_INCLUDE_DIR}")
endif(PMDSDK_FOUND)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PMDSDK_INCLUDE_DIR}
)

set(
  DEPENDENCIES  
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PMDSDK_LIBRARIES}
)

add_definitions(
  ${PCL_DEFINITIONS}
)
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

set(
  SOURCES 
  Object3D.cpp 
  Hand.cpp Plane.cpp 
  Webcam.cpp 
  DepthCamera.cpp 
  RGBCamera.cpp
  UDPSender.cpp 
  StreamingAverager.cpp 
  global.cpp 
  Calibration.cpp 
#  SensorIO.cpp 
  Util.cpp	
  Visualizer.cpp	
  main.cpp
  SR300Camera.cpp
  Converter.cpp
  PMDCamera.cpp
)

add_executable( OpenARK ${SOURCES} )
target_link_libraries( OpenARK ${DEPENDENCIES} )
set_target_properties( OpenARK PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

#add_dependencies(OpenARK ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS})





