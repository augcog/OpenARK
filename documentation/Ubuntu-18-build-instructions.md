# Installing OpenARK for Ubuntu 18.04.5 LTS (Bionic Beaver)

OpenARK provides support for Ubuntu 18.04.5 LTS (Bionic Beaver), 64-bit PC (AMD64) architecture.

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

## Installing PCL, [Package: libpcl-dev (1.8.1+dfsg1-2ubuntu2)](https://packages.ubuntu.com/bionic/libpcl-dev)

```sh
sudo apt -y install libpcl-*
```

## Installing OpenCV with Contrib

OpenCV 3.4.6, OpenCV_Contrib 3.4.6 and [Package: libopencv-dev (3.2.0+dfsg-4ubuntu0.1 and others)](https://packages.ubuntu.com/bionic/libopencv-dev)

1. Install prerequisites
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
make -j$(nproc)
sudo make install
```

## Fixing Eigen
The following sections directly have Eigen as a depenendency:
1. Ceres
2. OpenGV
3. Okvis
4. OpenARK

In order to get Eigen working for OpenARK Ubuntu and Jetson NX, the following lines of code must be added into the top level CMakesList.txt:
```
add_definitions(-DEIGEN_DONT_ALIGN=1)
add_definitions(-DEIGEN_DONT_VECTORIZE=1)
```
This will disable alignment as well as force the compiler to use c++17 standard.   

When it comes to OpenARK, the Eigen Fix is activated automatically if the host system is Linux in the CMakeLists.txt of OpenARK.

Then follow the CMake instructions to build each library.

## Continued Dependencies
## Installing Ceres 1.14.0 from source

### Installing [SuiteSparse 5.1.2](https://packages.ubuntu.com/bionic/libsuitesparse-dev)
SuiteSparse speeds up some of Ceres functions. However, if you want to prevent Suitesparse from crashing, you can skip downloading Suitesparse. You can still download Suitesparse and opt it out during the Ceres build process.
```sh
sudo apt install libsuitesparse-dev
```
### Download Ceres1.14.0 from source and build as follows. 
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
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Intalling OpenGV 1.0
1. `git clone https://github.com/laurentkneip/opengv && cd opengv`
2. Apply the Fixing Eigen changes.
3. Build with CMake and install
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Installing Brisk
1. `git clone https://github.com/sxyu/brisk && cd brisk`
2. Build with CMake and install
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Installing DBoW2 with Brisk Descriptors
1. `git clone https://github.com/joemenke/DBoW2_Mod && cd DBoW2_Mod`. Note that this repository is a modified version of DBoW2_Mod to support Brisk descriptors.
2. Build with CMake and install
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Installing DLoopDetector, Custom Version
1. `git clone https://github.com/joemenke/DLoopDetector && cd DLoopDetector`
Note that this repository is a modified version of DLoopDetector.
2. Build with CMake and install
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Installing Open3D (0.12.0)
1. `git clone --recursive https://github.com/moonwonlee/Open3D.git && cd Open3D`
2. `git checkout v12`
3. Install dependencies for Open3D.      
`./util/install_deps_ubuntu.sh assume-yes`
4. The minimum CMake version required is 3.18. You can check your CMake version with        
`cmake --version`     
For Ubuntu 18.04, we suggest you install the latest CMake from the official repository https://apt.kitware.com/.  CMake 3.18+ is required to allow linking with OBJECT libraries, to prevent erroneous -gencode option deduplication with CUDA, and to simplify generator expressions for selecting compile flags and setting global hardened link flags.
6. Build with CMake and install         
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=OFF -DGLIBCXX_USE_CXX11_ABI=ON ..
make -j$(nproc)
sudo make install
```

## Installing Okvis+ 
1. `git clone https://github.com/adamchang2000/okvis && cd okvis`
2. Apply the Fixing Eigen changes.
3. To run okvis demo, turn on the demo option in the CMakeListst.txt.      
`option (BUILD_APPS "Builds a demo app (which requires boost)" ON)` 
4. Build with CMake and install.
```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```
5. Verify Okvis+ by running the demo application.        
You will find a demo application in okvis_apps. It can process datasets in the ASL/ETH format.
In order to run a minimal working example, follow the steps below:

6. Download a dataset of your choice from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. Assuming you downloaded MH_01_easy/. You will find a corresponding calibration / estimator configuration in the config folder.

7. Run the app as
 `./okvis_app_synchronous path/to/okvis/config/config_fpga_p2_euroc.yaml path/to/mav0/`

## Installing librealsense2
Follow this : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
or here is summary.
``` sh
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```
(If using Ubuntu 16, replace `bionic` on the second line with `xenial`)

### Optional: As a sanity check for librealsense2, 
1. Connect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.
2. plugin your RealSense camera (SR300 or D400) and run `rs-capture` to see if streams appear.
3. Verify that the kernel is updated :
`modinfo uvcvideo | grep "version:" should include realsense string`

## Installing and Building OpenARK
1. Clone our repository: `git clone https://github.com/augcog/OpenARK && cd OpenARK`, or download the latest release.  
2. Build with CMake    
```sh
mkdir build && cd build

# Configure
# > librealsense2 will be enabled by default. 
# > Set `-DBUILD_AVATAR_DEMO` to build the avatar demo in addition to hand and SLAM
# > Set `-DBUILD_DATA_RECORDING` to build the data recording tool 
# > Set `-BUILD_TESTS` to build hand tests. 
# > `-DBUILD_UNITY_PLUGIN` is not available on Linux at the moment.
cmake .. -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
```

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
    meta.txt
    
Run as  `./OpenARK_slam_recording "/path/to/save/data" "/path/to/camera_yaml_file"`

Example `sudo ./OpenARK_slam_recording "../data" "../mycam_intr.yaml"`

### OpenARK_slam_replaying
Run as  `./OpenARK_slam_replaying "/path/to/camera_yaml_file path/to/brisk_vocab.bn" "0.0" "/path/to/data/collected/by/openark_slam_recording"`

Example `./OpenARK_slam_replaying "../mycam_intr.yaml" "../config/brisk_vocab.bn" "0.0" "../data_path"`

#### Dataset : [Google Drive](https://drive.google.com/drive/folders/1PyV8_0nDT9vURHWTvq-8RX9CV6gA2Abj?usp=sharing)
### 3dRecon_Data_Recording
Run as `./3dRecon_Data_Recording "/path/to/camera_yaml_file"`

Example `./3dRecon_Data_Recording "../config/d435i_intr.yaml"`

## Yaml File Explained
Please read the documentation of the individual parameters in the yaml file carefully. You have various options to trade-off accuracy and computational expense as well as to enable online calibration.

**Add these lines to your intrinsics yaml file**

    ```
    numKeypointsResetThreshold: 10
    durationResetThreshold: 0.5
    emitterPower: 0.
    ```

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
