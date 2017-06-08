cmake_minimum_required(VERSION 2.8)

## section: Macro
MACRO(ADD_MSVC_PRECOMPILED_HEADER PrecompiledHeader PrecompiledSource SourcesVar)
  IF(MSVC)
    GET_FILENAME_COMPONENT(PrecompiledBasename ${PrecompiledHeader} NAME_WE)
    SET(PrecompiledBinary "${CMAKE_CURRENT_BINARY_DIR}/${PrecompiledBasename}.pch")
    SET(Sources ${${SourcesVar}})

    SET_SOURCE_FILES_PROPERTIES(${PrecompiledSource}
                                PROPERTIES COMPILE_FLAGS "/Yc\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
                                           OBJECT_OUTPUTS "${PrecompiledBinary}")
    SET_SOURCE_FILES_PROPERTIES(${Sources}
                                PROPERTIES COMPILE_FLAGS "/Yu\"${PrecompiledBinary}\" /FI\"${PrecompiledBinary}\" /Fp\"${PrecompiledBinary}\""
                                           OBJECT_DEPENDS "${PrecompiledBinary}")
    # Add precompiled header to SourcesVar
    LIST(APPEND ${SourcesVar} ${PrecompiledSource})
  ENDIF(MSVC)
ENDMACRO(ADD_MSVC_PRECOMPILED_HEADER)

## start setting
SET (this_target OpenARKSDK)
PROJECT(${this_target})



## section: include directory
INCLUDE_DIRECTORIES(

  )

## section: source files
# Add your source files here (one file per line), please SORT in alphabetical order for future maintenance
SET (${this_target}_SOURCE_FILES
    ..\Clustering.cpp
	..\DepthSensor.cpp
	..\main.cpp
	..\PMDSensor.cpp
	..\RGBSensor.cpp
	..\Visualizer.cpp
	..\Sensor.cpp
	..\Webcam.cpp
    )

## section: header files
# Add your header files here(one file per line), please SORT in alphabetical order for future maintenance!
SET(${this_target}_HEADER_FILES
    ..\Clustering.h
	..\DepthSensor.h
	..\Hand.h
	..\PMDSensor.h
	..\RGBSensor.h
	..\Sensor.h
	..\Visualizer.h
	..\Webcam.h
    )

SOURCE_GROUP("Source Files" FILES
	..\main.cpp
	..\Sensor.cpp
	..\DepthSensor.cpp
	..\PMDSensor.cpp
	..\Visualizer.cpp
	..\RGBSensor.cpp
	..\Webcam.cpp
	..\Clustering.cpp
	)
SOURCE_GROUP("Header Files" FILES
	..\Sensor.h
	..\DepthSensor.h
	..\PMDSensor.h
	..\Visualizer.h
	..\RGBSensor.h
	..\Webcam.h
	..\Clustering.h
	..\Hand.h
	)
SOURCE_GROUP("Resource Files" FILES

	)


## section: precompiled header
#ADD_MSVC_PRECOMPILED_HEADER("precompiled.h" "precompiled.cpp" MySources)
#ADD_LIBRARY(MyLibrary ${MySources})

SET_SOURCE_FILES_PROPERTIES(${this_target}_HEADER_FILES
                            PROPERTIES HEADER_FILE_ONLY TRUE)
LIST(APPEND ${this_target}_SOURCE_FILES ${${this_target}_HEADER_FILES})

## section: add definitions
#   add prefix -D. example> -DSHP
#  - DO NOT add  the following definitions(already defined in ${OSP_DEFINITIONS}:
#   -DSHP, -DWIN32, -D_WINDOWS, -D_DEBUG, -D_USRDLL, -D_CRT_SECURE_NO_DEPRECATE
ADD_DEFINITIONS(

    )

## section: add target
