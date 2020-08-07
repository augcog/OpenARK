# Build Instructions for Windows Using Pre-Built Dependencies

Due to the complexity of the build process, we have prepared a Windows package that installs pre-built binaries for OpenARK dependencies. This should dramatically shorten setup time.

1. You will still need to install Visual Studio 2015 (vc14) from Microsoft: <https://visualstudio.microsoft.com/vs/older-downloads/>. Note that we built all dependencies in vc14, and newer VS versions (2017, 2019) will not work.

2. Install CMake: <https://cmake.org/download>. Make sure to select the option to add to system PATH during installation.

3. Download the PCL library from <https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2015-win64.exe>. Make sure to select the option to add to system PATH during installation. Note that this also installs Boost and Eigen (which will be duplicated by openark deps).
    - OpenNI2 will be installed as part of this installer. You will need to add the OpenNI2 dlls to the
system path manually. The default location these are installed to is C:\Program
Files\OpenNI2\Tools. See Step 4 in the opencv install instructions below for more details on
modifying system variables.

4. Download our installer from
[here](https://drive.google.com/file/d/1ef9PmvhlJGX5BH8LLcKaQxrY8PpTbcaW/view?usp=sharing) and run it. This should install pre-built versions for all other dependencies and configure CMake automatically.

5. In order to properly run OpenARK, you need to register connected Realsense devices. Instructions can be found at <https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_windows.md>. The `/scripts` directory has been included in the `librealsense2` installed in the dependency pack.

6. Now clone OpenARK, `cd` into the directory in a terminal, `mkdir build && cd build` and enter `cmake .. -G"Visual Studio 14 2015 Win64"`. This will take a while the first time as it will download several large data files from Github.

7. Optionally, manually set the `OPENARK_DIR` environment variable to the project directory. This allows OpenARK binaries to find the `config` and `data` files regardless of the current working directory.

8. Open `OpenARK.sln` and build as usual. (Note that only `Release` mode is supported)
