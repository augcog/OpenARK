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
[here](../openark-deps-vc14-win64.exe) and run it. This should install pre-built versions for all other dependencies and configure CMake automatically.

5. In order to properly run OpenARK, you need to register connected Realsense devices. Instructions can be found at <https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_windows.md>. The `/scripts` directory has been included in the `librealsense2` installed in the dependency pack.

6. Now clone OpenARK, `cd` into the directory in a terminal, `mkdir build && cd build` and enter `cmake .. -G"Visual Studio 14 2015 Win64"`. This will take a while the first time as it will download several large data files from Github.

7. Optionally, manually set the `OPENARK_DIR` environment variable to the project directory. This allows OpenARK binaries to find the `config` and `data` files regardless of the current working directory.

8. You can now either open the Visual Studio Solution generated in the build directory labeled “OpenARK.sln” or continue to build using the command prompt (Note that only `Release` mode is supported)

**Building via Visual Studio:**


 Right click on the OpenARK project in the solution explorer and select “Set as Startup Project”. Build and run as usual.

**Building via Command Prompt:** 

    `cmake --build . --config Release`

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
