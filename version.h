#pragma once

#define RSSDK_ENABLED
#define OPENARK_CAMERA_TYPE "sr300"
//#define PMDSDK_ENABLED
//#define OPENARK_CAMERA_TYPE "pmd" 

#define OPENARK_VERSION_MAJOR 0
#define OPENARK_VERSION_MINOR 9
#define OPENARK_VERSION_PATCH 1

// Uncomment to enable debug code
// #define DEBUG

namespace ark {
    // OpenARK version number (modify in CMakeLists.txt)
    static const char * VERSION = "0.9.1";

    // Paths to possible SVM model file locations (terminated by "\n")
    static const char * SVM_PATHS[] = {"svm/", "../svm/", "", "\n"};
}
