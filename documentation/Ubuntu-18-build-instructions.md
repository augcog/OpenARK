
## Installing OpenARK for Ubuntu 18.04.5 LTS (Bionic Beaver)

### Preliminaries

1. Install basic tools, dependencies:

```sh
sudo apt update
sudo apt -y install g++ build-essential cmake cmake-gui git-core
sudo apt -y install pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
sudo apt -y install mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
sudo apt -y install freeglut3-dev pkg-config libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete libflann-dev
```

2. `g++ --version` to check the g++ version. If it is less than 5, use the below commands to update GCC to version 5:

```sh
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
sudo update-alternatives --set gcc "/usr/bin/gcc-5"
```

### Note: CMake Builds
For conciseness, in all sections below "build with CMake" will mean ...

```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
```
or a one-liner for cmake build is ...
```sh
mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=RELEASE .. && make -j4 && sudo make install
```
Note that this installs the library. You may replace '4' in step 3 with any number of threads. The build process should not take too long.

### Installing PCL, [Package: libpcl-dev (1.8.1+dfsg1-2ubuntu2)](https://packages.ubuntu.com/bionic/libpcl-dev)

```sh
sudo apt -y install libpcl-*
```

### Installing OpenCV with Contrib

OpenCV 3.4.6, OpenCV_Contrib 3.4.6 and [Package: libopencv-dev (3.2.0+dfsg-4ubuntu0.1 and others)](https://packages.ubuntu.com/bionic/libopencv-dev)

1. Install prerequisites:

```sh
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt -y install libopencv-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper1 
sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt -y install libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
sudo apt -y install libvorbis-dev libxvidcore-dev v4l-utils
sudo apt -y install liblapacke-dev libopenblas-dev libgdal-dev checkinstall
sudo apt -y install libssl-dev libopenexr-dev openexr
sudo apt -y install libprotobuf-dev protobuf-compiler
sudo apt -y install libgoogle-glog-dev libgflags-dev
sudo apt -y install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
sudo apt -y install libgtk2.0-dev
sudo apt -y install libglfw3 libglfw3-dev
```

Note that we add the Ubuntu 16 (Xenial) repo since some packages have been removed in later versions of Ubuntu.

2. Download OpenCV and OpenCV_contrib sources:
```sh
wget -O opencv346.tar.gz https://github.com/opencv/opencv/archive/3.4.6.tar.gz
tar -xf opencv346.tar.gz
cd opencv-3.4.6
wget -O contrib.tar.gz https://github.com/opencv/opencv_contrib/archive/3.4.6.tar.gz
tar -xf contrib.tar.gz
```

3. Build:

``` sh
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -DWITH_TBB=ON -DOPENCV_EXTRA_MODULES_PATH="../opencv_contrib-3.4.6/modules" -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF ..
make -j4
sudo make install
```
Again, -j4 may be replaced with any number of threads.

## Fixing Eigen
The following sections directly have Eigen as a depenendency:
1. Ceres
2. OpenGV
3. Okvis
4. OpenARK

In order to get Eigen working, the following lines of code must be added into the top level CMakesList.txt:
```
message(STATUS "using c++17")
add_definitions(-DEIGEN_DONT_ALIGN=1)
add_definitions(-DEIGEN_DONT_VECTORIZE=1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17")
set(CMAKE_CXX_STANDARD 17)
```

This will disable alignment as well as force the compiler to use c++17 standard.  
Add these lines before running the Cmake instructions to build each library.

## Continued Dependencies
### Installing Ceres 1.14.0 from source

#### Installing [SuiteSparse 5.1.2](https://packages.ubuntu.com/bionic/libsuitesparse-dev)
SuiteSparse speeds up some of Ceres functions. However, if you want to prevent Suitesparse from crashing, you can skip downloading Suitesparse. You can still download Suitesparse and opt it out during the Ceres build process.
```sh
sudo apt install libsuitesparse-dev
```
Download Ceres1.14.0 from source and build as follows. 
1. `wget -O ceres114.tar.gz https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz && tar -xf ceres114.tar.gz && cd ceres-solver-1.14.0`
2. Comment out lines 434-439 of the top level CMakeLists.txt
It should look like: (notice the #[[ open comment and ]] close comment)
```
#[[if (CXX11 AND COMPILER_HAS_CXX11_FLAG)
  # Update CMAKE_REQUIRED_FLAGS used by CheckCXXSourceCompiles to include
  # -std=c++11 s/t we will detect the C++11 versions of unordered_map &
  # shared_ptr if they exist.
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -std=c++11")
endif (CXX11 AND COMPILER_HAS_CXX11_FLAG)]]
```
3. Apply the Fixing Eigen changes.
4. Build with CMake and install

### Intalling OpenGV 1.0
1. `git clone https://github.com/laurentkneip/opengv && cd opengv`
2. Apply the Fixing Eigen changes.
3. Build with CMake and install

### Installing Brisk

1. `git clone https://github.com/sxyu/brisk && cd brisk`
2. Build with CMake and install

### Installing DBoW2 with Brisk Descriptors, Custom Version

1. `git clone https://github.com/joemenke/DBoW2_Mod && cd DBoW2_Mod`. 
Note that this repository is a modified version of DBoW2_Mod to support Brisk descriptors.
2. Build with CMake and install

### Installing DLoopDetector, Custom Version

1. `git clone https://github.com/joemenke/DLoopDetector && cd DLoopDetector`
Note that this repository is a modified version of DLoopDetector.
2. Build with CMake and install

### NOTE: CURRENTLY OPEN3D isnt confirmed to work, 3drecondemo probably wont work.

### Installing [Open3D 0.8.0, Custom Version](https://github.com/adamchang2000/Open3D)
Note that this is a modified version of Open3D 0.8.0

1. `git clone --recursive https://github.com/moonwonlee/Open3D.git && cd Open3D`
or get `git clone --recursive https://github.com/adamchang2000/Open3D.git && cd Open3D`, go to src/Open3D/ and delete MovingTSDFVolume.cpp and MovingTSDFVolume.h in // this to be deleted later when Open3D is updated.
2. Build with CMake 
3. Make the following changes to Open3DConfig.cmake in Open3D/build/CMakeFiles // this to be deleted later when Open3D is updated or OpenARK CMakeLists.txt is updated.
```sh
Change:
From : get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
To   : get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)
  
Change:
From:
set(Open3D_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/Eigen;/usr/include/libdrm;/usr/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/fmt/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/fmt/include/format.h";${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/fmt/include/)
To:
set(Open3D_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include;${PACKAGE_PREFIX_DIR}/3rdparty/Eigen;/usr/include/libdrm;/usr/include;${PACKAGE_PREFIX_DIR}/3rdparty/fmt/include;${PACKAGE_PREFIX_DIR}/3rdparty/fmt/include/format.h")
```

4. `make -j4 && sudo make install`

### Installing Okvis+ 
1. `git clone https://github.com/adamchang2000/okvis && cd okvis`
Note that this is a modified version of Okvis.   
This already has the Eigen changes applied.
2. Build with CMake and install
3. Verify Okvis+ by running the demo application
You will find a demo application in okvis_apps. It can process datasets in the ASL/ETH format.
https://github.com/ceres-solver/ceres-solver/releases/tag/1.14.0
In order to run a minimal working example, follow the steps below:

4. Download a dataset of your choice from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. Assuming you downloaded MH_01_easy/. You will find a corresponding calibration / estimator configuration in the config folder.

5. Run the app as
 `./okvis_app_synchronous path/to/okvis/config/config_fpga_p2_euroc.yaml path/to/mav0/`
 
### Installing librealsense2
Follow this : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
or here is summary.
``` sh
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```
(If using Ubuntu 16, replace `bionic` on the second line with `xenial`)

#### Optional: As a sanity check, 
1. Connect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.
2. plugin your RealSense camera (SR300 or D400) and run `rs-capture` to see if streams appear.
3. Verify that the kernel is updated :
`modinfo uvcvideo | grep "version:" should include realsense string`

### Installing and Building OpenARK
1. Clone our repository: `git clone https://github.com/augcog/OpenARK`, or download the latest release.  
This already has the Eigen changes.

3. `cd OpenARK && mkdir build && cd build` to create build directory.

3.1 Add the following lines in CMakeLists.txt of OpenARK // this to be deleted later when OpenARK's CMakeLists.txt is updated.
``` sh
set(ENV{DLoopDetector_INCLUDE_DIRS} "/usr/local/include/DLoopDetector")
set(ENV{Open3D_DIR} "/home/[path_between_home_and_Open3D]/Open3D/build/CMakeFiles")
``` 
3.2 Fix the following lines in CMAkeLists.txt of OpenARK // this to be deleted later when OpenARK's CMakeLists.txt is updated.
``` sh
  From: find_package( OpenGV REQUIRED NO_MODULE )
  To  : find_package( opengv REQUIRED NO_MODULE )
  
  From: find_package( OpenGV REQUIRED )
  To  : find_package( opengv REQUIRED )
``` 
4. Do the following things before cmake build. // this to be deleted later when OpenARK's codes are updated.
``` sh
Open OpenARK/MockD435iCamera.cpp and delete & on the LHS of line 31:30 and 37:28
```
``` sh
Open OpenARK/SaveFrame.cpp and change line 17 to #include <boost/filesystem.hpp> 
Open OpenARK/SaveFrame.cpp and change mkdir(folderPath.c_str()); to boost::filesystem::create_directories(folderPath.c_str());
Open OpenARK/saveFrame.cpp and delete “SaveFrame::” part on line 23,24,25,27 in SaveFrame.h file
```
``` sh
Open OpenARK/include/SegmentedMesh.h and change
From :
#include "Open3D/geometry/PointCloud.h"
#include "Open3D/geometry/TriangleMesh.h"
To: 
#include "Open3D/Geometry/PointCloud.h"
#include "Open3D/Geometry/TriangleMesh.h"
```
``` sh
Open OpenARK/include/SegmentedMesh.h:9:50 and change
From : Line 9 : #include "Open3D/camera/PinholeCameraIntrinsic.h"
To   : Line 9 : #include "Open3D/Camera/PinholeCameraIntrinsic.h"
```
``` sh
Open OpenARK/SegmentedMesh.cpp:320:3 and change
From : line 320 : cout << "writing meshes" << endl;
To   : line 320:  std::cout << "writing meshes" << std::endl;
```
``` sh
Open OpenARK/include/glfwManager.h and change
From : line 221 : void MeshWindow::set_camera(Eigen::Affine3d t) {
To   : line 221 : void set_camera(Eigen::Affine3d t) {
```
``` sh
From : line 230 : MeshWindow::bool clicked() {
To   : line 230 : bool clicked() {
```
``` sh
Open OpenARK/SlamRecording.cpp:91:31 and change
From :
Line 91: std::atomic_bool paused = true;
Line 92: std::atomic_bool quit = false;    

To : line 91, 92:
Line 91: std::atomic_bool paused = {true};
Line 92: std::atomic_e bool quit = {false};
```

5. `cmake .. -DCMAKE_BUILD_TYPE=Release` to run CMake. librealsense2 will be enabled by default. You can add `-DBUILD_AVATAR_DEMO` to build the avatar demo in addition to hand and SLAM, `-DBUILD_DATA_RECORDING` to build the data recording tool, and `-BUILD_TESTS` to build hand tests. `-DBUILD_UNITY_PLUGIN` is not available on Linux at the moment.

6. `make -j4` to build.

7. As a final sanity check, try running the demo executable in the build directory: `./OpenARK-Hand-Demo`. You should see the hand detection and depth image windows. If you only see one window, drag it and see if the other is behind it. The static library is named: `libopenark_0_9_3.a`.


