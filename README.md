# OpenARK

OpenARK is an open-source wearable augmented reality (AR) system founded at UC Berkeley in 2016. The C++ based software offers innovative core functionalities to power a wide range of off-the-shelf AR components, including see-through glasses, depth cameras, and IMUs. The open-source platform includes fundamental tools such as AR-based camera calibration and SLAM, and it also includes higher-level functions to aid human-computer interaction, such as 3D gesture recognition and multi-user collaboration. Currently, it supports both PMD Pico Flexx and Intel RealSense SR300 cameras.

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
- OpenCV 3.2
- PCL 1.8
- OpenNI 1.5.8
- Boost 1.6.4

## Installation

1. Download and install all software depedencies (OpenCV, PCL, OpenNI, Boost, Intel RealSense)
2. Clone repo to local machine
3. Open the Visual Studios solution (OpenARK.sln)

## Configuration

Configure project properties (see /documentation/OpenARK_Setup.pdf)

## Usage

OpenARK is made for easy customization. Please feel free to build on top of this platform to fit your needs.

## How to test the software

Code used to run the demo video is included in main.cpp. Additional sample code can be found in /samplecode/.

## Known issues

OpenCV prior to 3.2.0 does not offer prebuilt VC14+ binaries. Running VC12 OpenCV binaries with VC14 will result in memories errors in findCountours(). If you are using VC12+ to compile OpenARK, you will need to use CMake to rebuilt OpenCV from source.
We have used OpenCV 3.2.0 which comes with OpenCV binaries for VC14 to avoid this problem.

## Getting help

If you have questions, concerns, bug reports, etc, please file an issue in this repository's Issue Tracker.

## Getting involved

The Center for Augmented Cognition welcome interested industry partners to join our alliance to support the Open ARK platform. More information can be found on cac.berkeley.edu

----

## Credits and references

Bill Zhou, Allen Yang, S.Shankar, Will Huang, Larry Yang, Eric Nyugen, Michelli Ni, Peter Li, Jessica Jiang, Mona Jalal.
