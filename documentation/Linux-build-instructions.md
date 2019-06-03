## OpenARK CMAKE build instructions for Ubuntu 16.04:

Due to the variety of distributions and packaging tools available, we will only provide detailed step-by-step build instructions for *Ubuntu 16.04* using `apt-get`. However, the steps below should be similar for any Linux distribution.

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

### Installing Boost, Flann
Simply: `sudo apt -y install libflann1.8 libflann-dev libboost1.58-all-dev` to install.

### Installing Eigen

```sh
wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
sudo apt-mark hold libeigen3-dev
```

### Installing VTK

We will build VTK from source.

```sh
wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz`
tar -xf VTK-7.1.0.tar.gz
cd VTK-7.1.0
```
Now build with CMake.

### Installing Glog

1. `git clone https://github.com/google/glog`

2. As usual, build with CMake

### Installing SuiteSparse

```sh
sudo add-apt-repository ppa:jmaye/ethz
sudo apt update
sudo apt install libsuitesparse-dev
```

### Installing OpenCV with Contrib

1. Install prerequisites:

```sh
sudo apt -y install libopencv-dev libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev
sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt -y install libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
sudo apt -y install libvorbis-dev libxvidcore-dev v4l-utils
sudo apt -y install liblapacke-dev libopenblas-dev libgdal-dev checkinstall
sudo apt -y install libssl-dev libopenexr-dev openexr
```

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
cmake -D CMAKE_BUILD_TYPE=RELEASE -DWITH_TBB=ON -DOPENCV_EXTRA_MODULES_PATH="../opencv_contrib-3.4.6/modules" ..
make -j4
sudo make install
```
Again, -j4 may be replaced with any number of threads.

### Installing PCL

We will build PCL from source. Please be warned that building VTK and PCL can take a while.
Some PCL dependencies, QHull and OpenNI, are optional and are not currently needed for building OpenARK. They will not be included in the build.

```sh
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
tar -xf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0
```
Then build with CMake as usual.

### Installing Ceres

1. `git clone https://github.com/joemenke/ceres-solver`

2. Build with CMake and install

### Intalling OpenGV

1. `git clone https://github.com/joemenke/opengv`

2. Build with CMake and install

### Installing Brisk

1. `git clone https://github.com/joemenke/brisk`

2. *Important*: `git checkout b9d5830`

3. Build with CMake and install

### Installing DBoW2 with Brisk Descriptors

1. `git clone https://github.com/joemenke/DBoW2_Mod`. Note that this repository is modified to support Brisk descriptors.

2. Build with CMake and install

### Installing DLoopDetector

1. `git clone https://github.com/joemenke/DLoopDetector`

2. Build with CMake and install

### Installing Okvis+

1. `git clone https://github.com/joemenke/okvis`. Note that this is a modified version of Okvis.

2. Build with CMake and install

### Installing RealSense SDK

OpenARK currently supports Intel's new RealSense cross-platform SDK (librealsense2): <https://github.com/IntelRealSense/librealsense/> on Linux.

First, try installing Intel's packages: 
``` sh
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt update
sudo apt -y install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```
If this installs librealsense2 correctly, then you are done. Else, proceed to the installation steps below:

1. We recommend installing Linux kernel image 4.13.0: `sudo apt -y install linux-tools-4.13.0-36-generic`. Alternatively, install
   kernel version 4.4.0 or 4.10.0 and run an Intel kernel patch (see step 5)

2. Reboot: `sudo update-grub && sudo reboot`

3. Get source by cloning the Github repo: `git clone https://github.com/IntelRealSense/librealsense.git`

4. `cd librealsense` and install udev rules:

```sh
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

5. If in step 1 you did not install 4.13.0, then apply the kernel patch by using: `./scripts/patch-realsense-ubuntu-xenial.sh`. This may or may not work, and you should use `dmesg | tail -n 50` to check. We recommend simply switching to 4.13.0.

6. Install required packages: `sudo apt -y install libusb-1.0-0-dev pkg-config libgtk-3-dev`

7. Make sure you are in the `librealsense` directory. Create the build directory: `mkdir build && cd build`

8. Run CMake: `cmake ../ -DBUILD_EXAMPLES=true`

9. Build and install: `sudo make uninstall && make clean && make -j4 && sudo make install` 

10. As a sanity check, plugin your RealSense camera (SR300 or D400) and run `rs-capture`. If a window shows up with the depth and color streams, congratulations! Otherwise, if there is an error, try installing different Linux kernel versions and patches (You can try running `./scripts/patch-realsense-ubuntu-xenial-joule.sh`) and repeat steps 8-10 until it works.

### Bulding OpenARK

1. Clone our repository: `git clone https://github.com/augcog/OpenARK`, or download the latest release.

2. `cd OpenARK && mkdir build && cd build` to create build directory.

3. `cmake ..` to run CMake. librealsense2 will be enabled by default. You can add `-DBUILD_AVATAR_DEMO` to build the avatar demo in addition to hand and SLAM, `-DBUILD_DATA_RECORDING` to build the data recording tool, and `-BUILD_TESTS` to build hand tests. `-DBUILD_UNITY_PLUGIN` is not available on Linux at the moment.

4. `make -j4` to build.

5. As a final sanity check, try running the demo executable in the build directory: `./OpenARK-Hand-Demo`. You should see the hand detection and depth image windows. If you only see one window, drag it and see if the other is behind it. The static library is named: `libopenark_0_9_3.a`.


