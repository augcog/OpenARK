# OpenARK

OpenARK is an open-source wearable augmented reality (AR) system founded at UC Berkeley in 2016. The C++ based software offers innovative core functionalities to power a wide range of off-the-shelf AR components, including see-through glasses, depth cameras, and IMUs. The open-source platform includes fundamental tools such as AR-based camera calibration and SLAM, and it also includes higher-level functions to aid human-computer interaction, such as 3D gesture recognition and multi-user collaboration. Currently, it supports both PMD Pico Flexx and Intel RealSense SR300 cameras. OpenARK currently only supports Windows and we have tested our platform with Windows 10 and Visual Studio 2015 Community Edition.

At a Glance

  - **Technology stack**: C++, OpenCV, PCL, Boost, OpenNI, RealSense 3D SDK, PMD SDK
  - **Status**:  Beta 0.8
  - **Application Demo**: vimeo.com/205084929



## Dependencies
Hardware
- Depth Camera
- RGB Camera
- Transparent AR Glasses (optional)

Software
- OpenCV 3.2.0
- PCL 1.8
- OpenNI 1.5.8
- Boost 1.6.4

## Installation

1. Download and install all software depedencies (OpenCV, PCL, OpenNI, Boost, Intel RealSense)
2. Clone repo to local machine
3. Open the Visual Studios solution (OpenARK.sln)
4. If you want to test the OpenARK using OpenARK_test following the given instruction in its repository.

## Configuration

Configure project properties (see /documentation/OpenARK_Setup.pdf)

## Usage

OpenARK is made for easy customization. Please feel free to build on top of this platform to fit your needs.

## How to test the software

Code used to run the demo video is included in main.cpp. Additional sample code can be found in /samplecode/ and you would need to replace it with the main that comes with the project solution.

## Known issues

OpenCV prior to 3.2.0 does not offer prebuilt VC14+ binaries. Running VC12 OpenCV binaries with VC14 will result in memories errors in findCountours(). If you are using VC12+ to compile OpenARK, you will need to use CMake to rebuilt OpenCV from source.
We have used OpenCV 3.2.0 which comes with OpenCV binaries for VC14 to avoid this problem when using Visual Studio 2015.

If you use `using namespace Intel::RealSense;` you will receive the error `Hand::Ambiguous symbol` hence when accessing methods and class members of RealSense use `Intel::RealSense`


## Getting help

If you have questions, concerns, bug reports, etc, please file an issue in this repository's Issue Tracker.

## Getting involved

The Center for Augmented Cognition welcomes interested industry partners to join our alliance to support the OpenARK platform. More information can be found on cac.berkeley.edu

----

## Credits and references

[Bill Zhou](http://www.billzhou.me/), [Allen Y. Yang](https://people.eecs.berkeley.edu/~yang/), [S. Shankar Sastry](http://robotics.eecs.berkeley.edu/~sastry/), [Will Huang](https://www.linkedin.com/in/hwjwill/), [Larry Yang](https://www.linkedin.com/in/larry-yang-33bab1aa/), [Eric Nguyen](https://www.linkedin.com/in/eric-nguyen-71577678/), Michelli Ni, [Peter Li](https://www.linkedin.com/in/peter-li-a770ab88/), Jessica Jiang, [Mona Jalal](http://monajalal.com/), [Joseph Menke](https://people.eecs.berkeley.edu/~joemenke/).
