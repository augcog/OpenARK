## OpenARK CMAKE build instructions for Linux:

Due to the variety of distributions and packaging tools available, we will only provide detailed step-by-step build instructions for *Ubuntu 16.04* using `apt-get`. However, the steps below (installing OpenCV, installing PCL, installing RealSense SDK, building OpenARK) should be similar for any Linux distribution.

### Preparing

1. Install basic tools:

```sh
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
```

2. Update GCC to version 5:

```sh
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
sudo update-alternatives --set gcc "/usr/bin/gcc-5"
```

### Installing OpenCV 3

1. Install prerequisites:

```sh
sudo apt-get install -y libopencv-dev libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install -y libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
sudo apt-get install -y libvorbis-dev libxvidcore-dev v4l-utils vtk6
sudo apt-get install -y liblapacke-dev libopenblas-dev libgdal-dev checkinstall
sudo apt-get install -y libssl-dev
```

2. Download the OpenCV sources (Minimum version 3.2) from their website: <https://opencv.org/releases.html>

3. Unpack the source zip using `unzip`. 

4. Enter the extracted directory and enter the following:

``` sh
mkdir build
cd build/
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D FORCE_VTK=ON -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_CUBLAS=ON -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D WITH_GDAL=ON -D WITH_XINE=ON -D BUILD_EXAMPLES=ON ..
make -j4
```

You may replace '4' in the last line with any number of threads. The build process should not take too long.

5. `sudo make install` to install. You may be prompted for your password.

(Credits to: [https://github.com/BVLC/caffe/wiki/OpenCV-3.3-Installation-Guide-on-Ubuntu-16.04])


### Installing PCL

PCL does offer pre-compiled binaries for a number of distributions on their website. Unfortunately, however, the precompiled binaries are quite outdated and unreliable.

Thus, we will build it from scratch, starting from the dependencies. Please be warned that this is a rather time-consuming process.

#### Installing Boost

Simply: `sudo apt-get install -y libboost-all-dev` to install.


#### Installing Eigen

1. Download Eigen's source archive: [http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2]

2. `tar xf` to extract.

3. `cd` into the extracted directory and `mkdir build && cd build && cmake ..`

4. `make -j4`. Again, you may replace '4' with any number of threads.

5. `sudo make install`. You may be prompted for your password.

#### Installing FLANN

1. Download source archive: [http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip]

2. `unzip` to extract

3. `cd` into extracted directory and `mkdir build && cd build && cmake ..`

4. `make -j4`

5. `sudo make install`

#### Installing VTK

We already installed VTK while building OpenCV. Thus, you can try skipping this section and building
PCL directly. However, if that fails with a message saying that VTK is missing, come back here and follow the instructions below: 

1. Download the source archive (.tar.gz; not the data): <https://www.vtk.org/download/>

2. `tar xf` to extract.

3. `cd` into the extracted directory and `mkdir build && cd build && cmake ..`

4. `make -j4` (Note: VTK is rather large, so this will take quite a while.)

5. `sudo make install`

#### Building PCL From Source

The other PCL dependencies, QHull and OpenNI, are optional and are not currently needed for building OpenARK.

1. Download the source archive (tar.gz) from: <https://github.com/PointCloudLibrary/pcl/releases>

2. Enter `tar xf pcl-pcl-1.x.x.tar.gz` to extract the contents, filling in the version number as necessary.

3. `cd pcl-pcl-1.x.x && mkdir build && cd build` to create the build directory.

4. `cmake -DCMAKE_BUILD_TYPE=Release ..` to run CMake.

5. `make -j4` to build (will take some time)

6. `sudo make install`

### Installing RealSense SDK

OpenARK currently supports Intel's new RealSense cross-platform SDK (RSSDK2): <https://github.com/IntelRealSense/librealsense/> on Linux. Sadly, Intel's pre-built binary also seems to be broken as of March 2018. Thus we will be building from source:

1. We recommend installing Linux kernel image 4.13.0: `sudo apt-get install linux-tools-4.13.0-36-generic`. Alternatively, install
   kernel version 4.4.0 or 4.10.0 and run an Intel kernel patch (see step 5)

2. Reboot: `sudo update-grub && sudo reboot`

3. Get source by cloning the Github repo: `git clone https://github.com/IntelRealSense/librealsense.git`

4. `cd librealsense` and install udev rules:

```sh
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

5. If in step 1 you did not install 4.13.0, then apply the kernel patch by using: `./scripts/patch-realsense-ubuntu-xenial.sh`. This may or may not work, and you should use `dmesg | tail -n 50` to check. We recommend simply switching to 4.13.0.

6. Install required packages: `sudo apt-get install -y libusb-1.0-0-dev pkg-config libgtk-3-dev`

7. Make sure you are in the `librealsense` directory. Create the build directory: `mkdir build && cd build`

8. Run CMake: `cmake ../ -DBUILD_EXAMPLES=true`

9. Build and install: `sudo make uninstall && make clean && make -j4 && sudo make install` 

10. As a sanity check, plugin your RealSense camera (SR300 or D400) and run `rs-capture`. If a window shows up with the depth and color streams, congratulations! Otherwise, if there is an error, try installing different Linux kernel versions and patches (You can try running `./scripts/patch-realsense-ubuntu-xenial-joule.sh`) and repeat steps 8-10 until it works.

### Bulding OpenARK

1. Clone our repository: `git clone https://github.com/augcog/OpenARK`

2. `cd OpenARK && mkdir build && cd build` to create build directory.

3. `cmake ..` to run CMake. RSSDK2 will be enabled by default.

4. `make -j4` to build

5. As a final sanity check, try running the demo executable in the build directory: `./OpenARK`. You should see the hand detection and depth image windows. If you only see one window, drag it and see if the other is behind it. The static library is named: `libopenark_0_9_3.a`.


