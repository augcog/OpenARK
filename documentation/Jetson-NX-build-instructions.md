# Installing OpenARK for NVIDIA Jetson Xavier NX

OpenARK provides support for Nvidia Jetson Xavier NX.  
CPU : (6-core NVIDIA Carmel ARM®v8.2 64-bit CPU (arm64 or aarch64).    
OS  : L4T = {sample filesystem based on Ubuntu 18.04, Linux Kernel 4.9, bootloader, NVIDIA drivers, flashing utilities}

## Preliminaries

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

## Note: CMake Builds
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

## Installing PCL, [Package: libpcl-dev (1.8.1+dfsg1-2ubuntu2)](https://packages.ubuntu.com/bionic/libpcl-dev)

```sh
sudo apt -y install libpcl-*
```

## Installing OpenCV with Contrib, for Jetson NX.

OpenCV 3.4.6, OpenCV_Contrib 3.4.6 and [Package: libopencv-dev (3.2.0+dfsg-4ubuntu0.1 and others)](https://packages.ubuntu.com/bionic/libopencv-dev)

1. Install prerequisites
```sh
sudo apt -y install libopencv-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libtiff5-dev libpng-dev
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
sudo apt -y libx264-dev libx265-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran python3-devsudo 
sudo apt -y apt install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly 
sudo apt -y gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 
sudo apt -ygstreamer1.0-pulseaudio libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
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

## Eigen Hack for OpenARK Ubuntu and Jetson NX.
The following sections directly have Eigen as a depenendency:
1. Ceres 1.14.0
2. OpenGV
3. Okvis
4. OpenARK Ubuntu.

In order to get Eigen working for OpenARK Ubuntu and Jetson NX, the following lines of code must be added into the top level CMakesList.txt:
```
add_definitions(-DEIGEN_DONT_ALIGN=1)
add_definitions(-DEIGEN_DONT_VECTORIZE=1)
```
This will disable alignment as well as force the compiler to use c++17 standard.  

When it comes to OpenARK, make sure that the EIGEN_HACK option is ON. This will apply the same Eigen hack above.       
`option( EIGEN_HACK "EIGEN_HACK" ON)`

Now run the CMake instructions to build each library.

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

## Intalling OpenGV 1.0
1. `git clone https://github.com/laurentkneip/opengv && cd opengv`
2. Apply the Fixing Eigen changes.
3. Build with CMake and install

## Installing Brisk, for Jetson NX.
1. `git clone https://github.com/moonwonlee/brisk.git && cd brisk`
2. Build with CMake and install

## Installing DBoW2 with Brisk Descriptors, for Jetson NX.
1. `git clone https://github.com/moonwonlee/DBoW2_Mod.git && cd DBoW2_Mod`. Note that this repository is a modified version of DBoW2_Mod to support Brisk descriptors.
2. Build with CMake and install

## Installing DLoopDetector, Custom Version
1. `git clone https://github.com/joemenke/DLoopDetector && cd DLoopDetector`
Note that this repository is a modified version of DLoopDetector.
2. Build with CMake and install

## Installing Open3D (0.12.0), for Jetson NX.
### Install system dependencies
```
sudo apt-get update -y
sudo apt-get install -y apt-utils build-essential git cmake
sudo apt-get install -y python3 python3-dev python3-pip
sudo apt-get install -y xorg-dev libglu1-mesa-dev
sudo apt-get install -y libblas-dev liblapack-dev liblapacke-dev
sudo apt-get install -y libsdl2-dev libc++-7-dev libc++abi-7-dev libxi-dev
sudo apt-get install -y clang-7
```
### Build.
```
# Clone
git clone --recursive https://github.com/moonwonlee/Open3D.git
git checkout v12
#Install dependencies for Open3D. 
./util/install_deps_ubuntu.sh assume-yes
cd Open3D
mkdir build
cd build

# Configure
# > Set -DBUILD_CUDA_MODULE=ON if CUDA is available (e.g. on Nvidia Jetson)
# > Set -DBUILD_GUI=ON if OpenGL is available (e.g. on Nvidia Jetson)
# > We don't support TensorFlow and PyTorch on ARM officially
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_CUDA_MODULE=OFF \
    -DBUILD_GUI=OFF \
    -DBUILD_TENSORFLOW_OPS=OFF \
    -DBUILD_PYTORCH_OPS=OFF \
    -DBUILD_UNIT_TESTS=ON \
    -DCMAKE_INSTALL_PREFIX=~/open3d_install \
    -DPYTHON_EXECUTABLE=$(which python) \
    -DBUILD_PYTHON_MODULE=OFF \
    -DGLIBCXX_USE_CXX11_ABI=ON 
    ..

# Build C++ library
make -j$(nproc)

# Install C++ package (optional)
sudo make install

# You can compile Open3D with -DBUILD_CUDA_MODULE=ON and -DBUILD_GUI=ON and the Open3D GUI app should be functional. We support CUDA v10.x, but other versions should work as well. 

# Run Open3D GUI (optional, available on when -DBUILD_GUI=ON)
./bin/Open3D/Open3D
```
If the Open3D build system complains about CMake xxx or higher is required, get a CMake version higher than 3.18.
Follow this => Compile CMake from source : https://cmake.org/install/
 
## Installing Okvis+, for Jetson NX.
1. `git clone https://github.com/moonwonlee/okvis.git && cd okvis`
2. Apply the Fixing Eigen changes.
3. To run okvis demo, turn on the demo option in the CMakeListst.txt.      
`option (BUILD_APPS "Builds a demo app (which requires boost)" ON)` 
4. Build with CMake and install.
5. Verify Okvis+ by running the demo application.
You will find a demo application in okvis_apps. It can process datasets in the ASL/ETH format.
In order to run a minimal working example, follow the steps below:

6. Download a dataset of your choice from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. Assuming you downloaded MH_01_easy/. You will find a corresponding calibration / estimator configuration in the config folder.

7. Run the app as
 `./okvis_app_synchronous path/to/okvis/config/config_fpga_p2_euroc.yaml path/to/mav0/`

## Installing librealsense2, for Jetson NX.
Follow this : https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

### Optional: As a sanity check for librealsense2, 
1. Connect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.
2. plugin your RealSense camera (SR300 or D400) and run `rs-capture` to see if streams appear.
3. Verify that the kernel is updated :
`modinfo uvcvideo | grep "version:" should include realsense string`

## Installing and Building OpenARK, for Jetson NX.
1. Clone our repository: `git clone https://github.com/augcog/OpenARK`, or download the latest release.  
2. `git checkout jetson-nx`.   
This already has the Eigen changes. Make sure that the EIGEN_HACK option is ON.     
`option( EIGEN_HACK "EIGEN_HACK" ON)`.      
Also make sure that the following is applied for ARM support, in case it is not applied automatically.     
`add_definitions(-D__ARM_NEON__)`

3. `cd OpenARK && mkdir build && cd build` to create build directory.

4. `cmake .. -DCMAKE_BUILD_TYPE=Release` to run CMake. librealsense2 will be enabled by default. You can add `-DBUILD_AVATAR_DEMO` to build the avatar demo in addition to hand and SLAM, `-DBUILD_DATA_RECORDING` to build the data recording tool, and `-BUILD_TESTS` to build hand tests. `-DBUILD_UNITY_PLUGIN` is not available on Linux at the moment.

5. `make -j4` to build.

## Running the OpenARK demo applications
- These applications will show you how core functionalities of OpenARK work 
- These applications will also help you to verify successful installation of OpenARK.
- The demos apps are in `build` directory.
- If you do not want to create these demo files, you can do turn off building them in `CMakeLists.txt`.
`option( BUILD_HAND_DEMO "BUILD_HAND_DEMO" OFF )`

### OpenARK_hand_demo
As a final sanity check, try running the demo executable in the build directory: 
`./OpenARK-Hand-Demo` 
You should see the hand detection and depth image windows. If you only see one window, drag it and see if the other is behind it. The static library is named: `libopenark_0_9_3.a`.

### OpenARK_slam_demo
Run as  `./OpenARK_SLAM_demo "/path/to/camera_intr.yaml" "../config/brisk_vocab.bn" "0.0"`

Example `./OpenARK_SLAM_demo "../mycam_intr.yaml" "../config/brisk_vocab.bn" "0.0"`

### OpenARK_slam_recording
#### Data Collected
    depth
    infrared
    infrared2
    timestamp.txt
    imu.txt
    rgb
    intrin.bin
    meta.txt
    
Run as  `./OpenARK_slam_recording "/path/to/save/data" "/path/to/camera_yaml_file"`

Example `sudo ./OpenARK_slam_recording "../data" "../mycam_intr.yaml"`

### OpenARK_slam_replaying
Run as  `./OpenARK_slam_replaying "/path/to/camera_yaml_file path/to/brisk_vocab.bn" "0.0" "/path/to/data/collected/by/openark_slam_recording"`

Example `./OpenARK_slam_replaying "../mycam_intr.yaml" "../config/brisk_vocab.bn" "0.0" "../myroom4"`

### 3dRecon_data_recording

## Yaml File Explained
Please read the documentation of the individual parameters in the yaml file carefully. You have various options to trade-off accuracy and computational expense as well as to enable online calibration.

## Installation Errors
Linux/Ubuntu is case-sensitive.

## Debugging Tips
#### [Valgrind](https://www.valgrind.org/docs/manual/manual-core.html): 
This tool can help you to locate segfaults and invalid write/reads

Run as `G_SLICE=always-malloc G_DEBUG=gc-friendly  valgrind -v --tool=memcheck --leak-check=full --num-callers=40 --log-file=valgrind.log <application name> <argument 1> <argument 2> <argument 3> > output.txt 2>&1`

Example `G_SLICE=always-malloc G_DEBUG=gc-friendly  valgrind -v --tool=memcheck --leak-check=full --num-callers=40 --log-file=valgrind.log ./OpenARK_SLAM_demo > output.txt 2>&1`

#### gdb
    gdb OpenARK_SLAM_DEMO
    r
    bt
    
    gdb --args okvis_app_synchronous ../config/config_fpga_p2_euroc.yaml /home/moon/Desktop/mav0/
    r
    bt

#### Debugging with print statements
    printf("debug");
    fflush(stdout);


