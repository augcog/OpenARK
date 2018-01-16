#pragma once

#define RSSDK_ENABLED
#define OPENARK_CAMERA_TYPE "sr300"
//#define PMDSDK_ENABLED
//#define OPENARK_CAMERA_TYPE "pmd" 

// Remove to disable visualizations (if building as library)
#define DEMO

// Remove to disable debug code
#define DEBUG

// Remove to disable plane detection
#define PLANE_ENABLED

namespace ark {
    // OpenARK version number
    static const char * VERSION = "0.8.1";
}
