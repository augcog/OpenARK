FIND_LIBRARY(DLoopDetector_LIBRARY DLoopDetector
    PATHS C:/Program Files/DLoopDetector/lib
)
FIND_PATH(DLoopDetector_INCLUDE_DIR DLoopDetectorConfig.cmake
    PATHS C:/Program Files/DLoopDetector/include/DLoopDetector 
)
SET(DLoopDetector_LIBRARIES ${DLoopDetector_LIBRARY})
SET(DLoopDetector_LIBS ${DLoopDetector_LIBRARY})
SET(DLoopDetector_INCLUDE_DIRS ${DLoopDetector_INCLUDE_DIR})
