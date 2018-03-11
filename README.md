# OpenARK

OpenARK is an open-source wearable augmented reality (AR) system founded at UC Berkeley in 2016. The C++ based software offers innovative core functionalities to power a wide range of off-the-shelf AR components, including see-through glasses, depth cameras, and IMUs. The open-source platform includes fundamental tools such as AR-based camera calibration and SLAM, and it also includes higher-level modules to aid human-computer interaction, such as 3D gesture recognition, plane detection, and multi-user collaboration, all with real-time (30+ FPS) performance. Currently, it supports both PMD Pico Flexx and Intel RealSense SR300 cameras. The project builds natively on both Windows and Linux.

At a Glance

  - **Technology stack**: C++, OpenCV, PCL, Boost, Intel RealSense SDK (1 or 2) / PMD SDK
  - **Status**:  Beta 0.9.3
  - **Application Demo**: [Vimeo](https://vimeo.com/251436256)

## Dependencies
Hardware
- Depth Camera
- RGB Camera
- Transparent AR Glasses (optional)

Software
- OpenCV 3.2.0+
- PCL 1.8
- Boost 1.6.4

## Installation

### Building From Scratch

Instructions are available in the following documents:

- Windows: [documentation/Windows-build-instructions.pdf](https://github.com/augcog/OpenARK/blob/master/documentation/Windows-build-instructions.pdf)

- Linux: [documentation/Linux-build-instructions.md](https://github.com/augcog/OpenARK/blob/master/documentation/Linux-build-instructions.md)

### Prebuilt Binaries (For Windows)
Prebuilt binaries for 64-bit Windows are available [here](https://github.com/augcog/OpenARK/releases).
Both the static library (with the headers) and the demo program are included. For the SVM to work properly, the `svm/` folder
 must be present in the current directory OR under the directory pointed to by the OPENARK_DIR environment variable.

## Usage
To use OpenARK in a Visual Studio C++ project:

After setting up all the dependencies (OpenCV, PCL, etc.) according the configuration instructions, add `openark_x_x_x.lib` to `Project > MyProjectProperties > Linker > Input > Additional Dependencies`. Then add `OPENARK_DIR/include` to `C/C++ > General > Additional Include Directories`. Finally, make sure that under `C/C++ > Code Generation`, `Runtime Library` is set to `Multi-threaded DLL (/MD)`.

Now you can #include OpenARK's core header in any file and begin using OpenARK. You would probably also want to include one of the depth camera backend headers (e.g. `SR300Camera.h`).

Here is the outline of a program for performing hand detection:
```cpp
...
#include "core.h"
#include "SR300Camera.h"

#include "opencv2/core.hpp"
#include <vector>
...
int main() {
    ark::DepthCamera & camera = ark::SR300Camera(); // OpenARK camera backend
    ark::HandDetector detector(); // OpenARK hand detector; also see PlaneDetector
    
    // start the camera; alternatively, call nextFrame() manually inside the loop (slower)
    camera.beginCapture(); 
    ...
    while (true) {
        cv::imshow("XYZ Map", camera.getXYZMap());

        detector.update(camera);
        std::vector<ark::Hand::Ptr> hands = detector.getHands();
        
        // do something with the hands detected
        ...
        
        // quit when q is pressed
        if (cv::waitKey(1) == 'q') break;
    }
    
    ...
    // automatically stops capture on exit
}
```

Additional sample code is available in `samplecode/`.

## Customization

OpenARK is made for easy customization. Please feel free to build on top of this platform to fit your needs.

## How to test the software

Code used to run the demo video is included in main.cpp. Additional sample code can be found in /samplecode/ and you would need to replace it with the main that comes with the project solution.

## Known issues

OpenCV prior to 3.2.0 does not offer prebuilt VC14+ binaries. Running VC12 OpenCV binaries with VC14 will result in memories errors in findCountours(). If you are using VC12+ to compile OpenARK, you will need to use CMake to rebuilt OpenCV from source.
We have used OpenCV 3.2.0 which comes with OpenCV binaries for VC14 to avoid this problem when using Visual Studio 2015.

## Getting help

If you have questions, concerns, bug reports, etc, please file an issue in this repository's Issue Tracker.

## Getting involved

The Center for Augmented Cognition welcomes interested industry partners to join our alliance to support the OpenARK platform. More information can be found on cac.berkeley.edu

----

## Credits and references

[Bill Zhou](http://www.billzhou.me/), [Allen Y. Yang](https://people.eecs.berkeley.edu/~yang/), [S. Shankar Sastry](http://robotics.eecs.berkeley.edu/~sastry/), [Will Huang](https://www.linkedin.com/in/hwjwill/), [Larry Yang](https://www.linkedin.com/in/larry-yang-33bab1aa/), [Eric Nguyen](https://www.linkedin.com/in/eric-nguyen-71577678/), Michelli Ni, [Peter Li](https://www.linkedin.com/in/peter-li-a770ab88/), [Jessica Jiang](https://github.com/jessicajiang), [Mona Jalal](http://monajalal.com/), [Joseph Menke](https://people.eecs.berkeley.edu/~joemenke/),
Lawrence Chen, Kuan Lu, Rachel Lee, Justin Yang, [Alex Yu](https://alexyu.net).
