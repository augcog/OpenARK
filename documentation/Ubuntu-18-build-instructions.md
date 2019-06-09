## OpenARK CMAKE build instructions for Ubuntu 18.04 LTS:

Due to the variety of distributions and packaging tools available, we will only provide detailed step-by-step build instructions for *Ubuntu 18.04* using `apt`. However, the steps below should be similar for any Linux distribution.

### Preliminaries

1. Install basic tools, dependencies:

```sh
sudo apt update
sudo apt -y install g++ build-essential cmake cmake-gui git-core
sudo apt -y install pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
sudo apt -y install mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
sudo apt -y install freeglut3-dev pkg-config libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev
```

2. `g++ --version` to check the g++ version. If it is less than 5, use the below commands to update GCC to version 5:

```sh
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
sudo update-alternatives --set gcc "/usr/bin/gcc-5"
```

### NOTE: CMake Builds

For conciseness, in all sections below "build with CMake" will mean

```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
```

Note that this installs the library. You may replace '4' in step 3 with any number of threads. The build process should not take too long.

### Installing PCL

```sh
sudo apt install libpcl-*
```


### Installing SuiteSparse

```sh
sudo apt install libsuitesparse-dev
```

### Installing OpenCV with Contrib

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
sudo apt -y install libgoogle-glog libgoogle-glog-dev libgflags-dev
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

### Installing Ceres

1. `wget -O ceres114.tar.gz https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz && tar -xf ceres114.tar.gz && cd ceres-solver-1.14.0

2. Build with CMake and install

### Intalling OpenGV

1. `git clone https://github.com/laurentkneip/opengv && cd opengv`

2. Build with CMake and install

### Installing Brisk

1. `git clone https://github.com/sxyu/brisk && cd brisk`

2. Build with CMake and install

### Installing DBoW2 with Brisk Descriptors

1. `git clone https://github.com/joemenke/DBoW2_Mod && cd DBoW2_Mod`. Note that this repository is modified to support Brisk descriptors.

2. Build with CMake and install

### Installing DLoopDetector

1. `git clone https://github.com/joemenke/DLoopDetector && cd DLoopDetector`

2. Build with CMake and install

### Installing Okvis+

1. `git clone https://github.com/joemenke/okvis && cd okvis`. Note that this is a modified version of Okvis.

2. Build with CMake and install

### Installing librealsense2

OpenARK currently supports Intel's new RealSense cross-platform SDK (librealsense2): <https://github.com/IntelRealSense/librealsense/> on Linux.

We can now install Intel's packages directly: 
``` sh
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt update
sudo apt -y install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```
(If using Ubuntu 16, replace `bionic` on the second line with `xenial`)

Optional: As a sanity check, plugin your RealSense camera (SR300 or D400) and run `rs-capture` to see if streams appear.

### Bulding OpenARK

1. Clone our repository: `git clone https://github.com/augcog/OpenARK`, or download the latest release.

2. `cd OpenARK && mkdir build && cd build` to create build directory.

3. `cmake .. -DCMAKE_BUILD_TYPE=Release` to run CMake. librealsense2 will be enabled by default. You can add `-DBUILD_AVATAR_DEMO` to build the avatar demo in addition to hand and SLAM, `-DBUILD_DATA_RECORDING` to build the data recording tool, and `-BUILD_TESTS` to build hand tests. `-DBUILD_UNITY_PLUGIN` is not available on Linux at the moment.

4. `make -j4` to build.

5. As a final sanity check, try running the demo executable in the build directory: `./OpenARK-Hand-Demo`. You should see the hand detection and depth image windows. If you only see one window, drag it and see if the other is behind it. The static library is named: `libopenark_0_9_3.a`.


