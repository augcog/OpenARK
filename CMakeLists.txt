cmake_minimum_required(VERSION 2.8)

project( OpenARK )

set(OpenARK_VERSION_MAJOR 0)
set(OpenARK_VERSION_MINOR 9)
set(OpenARK_VERSION_PATCH 3)

set(SVM_PATHS "\"svm/\", \"../svm/\", \"\"")
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

set(CMAKE_CXX_STACK_SIZE "10000000")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

set(LIB_NAME "OpenARK")
set(DEMO_NAME "OpenARK_demo")
set(TEST_NAME "OpenARK_test")

option(BUILD_DEMO "BUILD_DEMO" ON)
option(BUILD_TESTS "BUILD_TESTS" OFF)
option(USE_PMD "USE_PMD" OFF) 

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

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_STATIC ON)

find_package( PCL REQUIRED )

find_package( OpenCV REQUIRED )
if(OpenCV_FOUND)
   message(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS}")
endif(OpenCV_FOUND)

if(NOT USE_PMD)
  find_package(RSSDK)
  if(RSSDK_FOUND)
       message(STATUS "Found RSSDK: ${RSSDK_INCLUDE_DIR}")
  endif(RSSDK_FOUND)
endif(NOT USE_PMD)

if(USE_PMD OR NOT RSSDK_FOUND)
  find_package( PMDSDK REQUIRED)
  if(PMDSDK_FOUND)
     message(STATUS "Found PMDSDK: ${PMDSDK_INCLUDE_DIR}")
  endif(PMDSDK_FOUND)
endif(USE_PMD OR NOT RSSDK_FOUND)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(
  DEPENDENCIES  
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_definitions(
  ${PCL_DEFINITIONS}
)
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

set(
  SOURCES 
  FrameObject.cpp 
  Hand.cpp FramePlane.cpp 
  Webcam.cpp 
  DepthCamera.cpp 
  RGBCamera.cpp
  StreamingAverager.cpp 
  Calibration.cpp 
  Util.cpp	
  Visualizer.cpp	
  Converter.cpp
  HandClassifier.cpp
  Detector.cpp
  HandDetector.cpp
  PlaneDetector.cpp
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
  ${INCLUDE_DIR}/StreamingAverager.h 
  ${INCLUDE_DIR}/Calibration.h 
  ${INCLUDE_DIR}/Util.h	
  ${INCLUDE_DIR}/Visualizer.h	
  ${INCLUDE_DIR}/Converter.h
  ${INCLUDE_DIR}/HandClassifier.h
  ${INCLUDE_DIR}/Detector.h
  ${INCLUDE_DIR}/HandDetector.h
  ${INCLUDE_DIR}/PlaneDetector.h
  stdafx.h
)

if(NOT RSSDK_FOUND)
  SET(RSSDK "//")
else()
  set(SOURCES ${SOURCES} SR300Camera.cpp)
  set(HEADERS ${HEADERS} ${INCLUDE_DIR}/SR300Camera.h)
endif(NOT RSSDK_FOUND)

if(NOT PMDSDK_FOUND)
  SET(PMDSDK "//")
else()
  include_directories(${PMDSDK_INCLUDE_DIR})
  set(SOURCES ${SOURCES} PMDCamera.cpp)
  set(HEADERS ${HEADERS} ${INCLUDE_DIR}/PMDCamera.h)
  set(DEPENDENCIES ${DEPENDENCIES} ${PMDSDK_LIBRARIES})
endif(NOT PMDSDK_FOUND)

configure_file(Version.h.in ${INCLUDE_DIR}/Version.h)
set(HEADERS ${HEADERS} ${INCLUDE_DIR}/Version.h)

# create source group
source_group("Header Files" REGULAR_EXPRESSION ".*\\.(h|hpp)")

macro(add_msvc_pch PrecompiledHeader PrecompiledSource SourcesVar)
  if(MSVC)
    get_filename_component(PrecompiledBasename ${PrecompiledHeader} NAME_WE)
    set(PrecompiledBinary "$(IntDir)/${PrecompiledBasename}.pch")
    set(Sources ${${SourcesVar}})

    if(NOT ${PrecompiledSource} STREQUAL "")
        set(Sources ${Sources} main.cpp)
        set_source_files_properties(${PrecompiledSource}
                                    PROPERTIES COMPILE_FLAGS "/Yc\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
                                               OBJECT_OUTPUTS "${PrecompiledBinary}")
    endif(NOT ${PrecompiledSource} STREQUAL "")

    set_source_files_properties(${Sources}
                                PROPERTIES COMPILE_FLAGS "/Yu\"${PrecompiledHeader}\" /FI\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
                                           OBJECT_DEPENDS "${PrecompiledBinary}")  
    list(APPEND ${SourcesVar} ${PrecompiledSource})
  endif(MSVC)
endmacro(add_msvc_pch)

if (MSVC)
    add_msvc_pch("stdafx.h" "stdafx.cpp" SOURCES)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /Ot")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
endif(MSVC)

add_library(${LIB_NAME} STATIC "${INCLUDE_DIR}/Core.h" ${SOURCES} ${HEADERS})
set_target_properties( ${LIB_NAME} PROPERTIES OUTPUT_NAME 
        "openark_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}")
set_target_properties(${LIB_NAME} PROPERTIES COMPILE_FLAGS "/MD")
set_property(TARGET ${LIB_NAME} PROPERTY DBG_POSTFIX d)
target_link_libraries( ${LIB_NAME} ${DEPENDENCIES} )
target_include_directories(${LIB_NAME} PRIVATE ${INCLUDE_DIR})

if(${BUILD_DEMO})
    add_executable( ${DEMO_NAME} main.cpp ${SOURCES} ${HEADERS})
    target_include_directories(${DEMO_NAME} PRIVATE ${INCLUDE_DIR})
    target_link_libraries( ${DEMO_NAME} ${DEPENDENCIES} )
    set_target_properties( ${DEMO_NAME} PROPERTIES OUTPUT_NAME ${LIB_NAME})
    set_target_properties( ${DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
endif(${BUILD_DEMO})

if(${BUILD_TESTS})
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/OpenARK_test)
    execute_process(
      COMMAND git clone https://github.com/augcog/OpenARK_test.git
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  endif(NOT EXISTS ${PROJECT_SOURCE_DIR}/OpenARK_test)

  set(
    TEST_SOURCES 
    OpenARK_test/test.cpp
    OpenARK_test/TestCamera.cpp
  )

  add_msvc_pch("../stdafx.h" "" TEST_SOURCES)

  add_executable( ${TEST_NAME} ${TEST_SOURCES} ${SOURCES} OpenARK_test/TestCamera.h ${HEADERS})
  target_link_libraries( ${TEST_NAME} ${DEPENDENCIES} )
  set_target_properties( ${TEST_NAME} PROPERTIES
    COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
    
  target_include_directories(${TEST_NAME} PRIVATE ${INCLUDE_DIR})
endif(${BUILD_TESTS})
