**OpenARK CMAKE Build Instructions for Windows:**

**Prerequisites:**

We recommend using Visual Studio 2015 and 64-bit Windows for OpenARK. When given the option please select Visual Studio 2015 (also known as vc14) and x64 for all prerequisite software.

1.  Install Visual Studio 2015. Select custom installation and make sure to install the C++ build tools under Programming Languages.
    
2. Install cmake: https://cmake.org/download/

    Make sure to select “add cmake to system PATH” either for current user or all users. 


3. Install PCL1.8, easiest way is to use the all-in-one installer made by a kind PCL user: http://unanancyowen.com/en/pcl18/#Download

    Make sure to select “add PCL to system PATH” either for current user or all users.

    OpenNI2 will be installed as part of this installer. You will need to add the OpenNI2 dlls to the system path manually. The default location these are installed to is C:\Program Files\OpenNI2\Tools. See Step 4 in the opencv install instructions below for more details on modifying system variables. 

4. **Clone OpenARK repo from https://github.com/augcog/OpenARK/**

 

**General CMAKE Build Instructions:**  
Make sure to run Administrator Command Prompt throughout the build process to avoid certain privilege warnings/errors.   
Refer to the end of this document for general troubleshooting strategies to help debug any errors you may run into.

Most of the following files will follow the general CMAKE build steps shown below:

1. Make a build directory to store build files: 
    `mkdir build`

2. Generate a Visual Studio solution: 
    `cd build` and then
   ` cmake -G"Visual Studio 14 2015 Win64" ..`

3. Build and install:
    `cmake --build . --config Release --target install`

 

**Installing All Dependencies**

Install opencv 3.4 and opencv_contrib from source:

1. Download opencv_contrib 3.4 source from https://github.com/opencv/opencv_contrib/tree/3.4. Extract the zip folder. 

2.    Download OpenCV 3.4 source from https://github.com/opencv/opencv/tree/3.4. Extract the zip folder and cd to the extracted directory. If the latest version of OpenCV 3.4 causes an error in the build process later on, use OpenCV 3.4.8 (proven to work correctly).   

3. Follow CMAKE steps above, EXCEPT replace `cmake -G"Visual Studio 14 2015 Win64" ..` with `cmake -G"Visual Studio 14 2015 Win64" -DOPENCV_EXTRA_MODULES_PATH=<opencv_contrib>/modules -DWITH_MSMF=OFF ..`
    Where: <opencv_contrib> is the location you extracted the opencv_contrib zip file to.

4.    Set the OpenCV enviroment variable and add it to the systems path. You can edit the environment variables using an interface by going to “Control Panel->System and Security->System->Advanced system settings->Environment Variables”. 

 We will need to modify two system variables in order to use OpenCV. First we want to add the system variable OpenCV_DIR and set it to `<extraction_directory>\opencv\Build\install` where `<extraction_directory>` is the directory you extracted OpenCV to. 

 You will also need to add the bin directory to the PATH variable. Select the variable “Path” and add a line with the value `%OpenCV_DIR%\x64\vc14\bin`. This allows your computer to find the proper library files when running programs using OpenCV. 

 Hit “OK” to close the Environment Variables dialog box. The variables will not be set until you do this. You will also have to reopen any open command prompt windows for the variables to be updated for that window. 

 

 

Install Eigen 3:

1.    Download source from http://eigen.tuxfamily.org/. Extract zip folder and cd to the extracted directory. 

2.    Follow CMAKE steps above.

Install Glog: 

1.    Download source from https://github.com/google/glog. Extract zip folder and cd to the extracted directory. 

2.    Follow CMAKE steps above.

Install SuiteSparse: 

1.    Download source from https://github.com/joemenke/suitesparse-metis-for-windows. Extract zip folder and cd to the extracted directory. (Note this is our forked version that builds with VS2015).

2.    Follow CMAKE steps above.

3.    You will need to add the “SuiteSparse_DIR” environment variable and set it to “<build dir from step 2>\install”

4.    You will also need to add the lapack install location to your Path. Open the Path environment variable and add a new row and set it to `<build dir from step 2>\install\lib64\lapack_blas_windows`

Install Ceres Solver:

1.    Download source from https://github.com/joemenke/ceres-solver. Extract zip folder and cd to the extracted directory.  (Note: this is simply a fork from the original git repository to make install on windows easier)

2.    Follow CMAKE steps above.

Install Boost:

1.    Download source from (boost location). Extract zip folder and cd to the extracted directory. 

2.    Set up:
    bootstrap

3.    Install:
    `.\b2 --toolset=msvc-14.0 --build-type=complete --prefix=C:\Boost architecture=x86 address-model=64 install`
    Note: You may get an error about the msvc version not matching. If so you need to modify project-config.jam with “using msvc : 14.0 ;”

4.    Boost doesn’t officially support Visual Studio 2015 (though it works fine), you need to modify `$(boost install location)/include/boost-1_XX/boost/config/auto_link.hpp` and change: 
 ```
 #eleif defined(BOOST_MSVC)
 #define BOOST_LIB_TOOLSET “vc120”
 ```
 to
 ```
 #eleif defined(BOOST_MSVC)
 #define BOOST_LIB_TOOLSET “vc140”
 ```

Install OpenGV:

1.    Download source from https://github.com/joemenke/opengv. Extract zip folder and cd to the extracted directory.  (Note: this is simply a fork from the original git repository to make install on windows easier)

2.    Follow CMAKE steps above.

3.    You will need to add the environment variable `OpenGV_DIR` and set it to the install location of OpenGV (probably C:\Program Files\opengv)

Install Brisk:

1.    Download source from https://github.com/joemenke/brisk. Extract zip folder and cd to the extracted directory. (Note: this is simply the brisk originally included with okvis, modified to make install on windows easier).

2.    Follow CMAKE steps above.

Install DBoW2:

1.    Download source from https://github.com/joemenke/DBoW2_Mod. Extract zip folder and cd to the extracted directory. (Note: this is a fork from the original git repository to make install on windows easier and add support for Brisk descriptors).

2.    Follow CMAKE steps above.

3.    You will need to add the environment variable `DBoW2_ROOT_DIR` and set it to the install location of DBoW2 (probably C:\Program Files\DBoW2) 

Install DLoopDetector:

1.    Download source from https://github.com/joemenke/DLoopDetector. Extract zip folder and cd to the extracted directory. (Note: this is a fork from the original git repository with some minor modifications to support OpenARK integration).

2.    Follow CMAKE steps above.

3.    Add enviromental variable `DLoopDetector_INCLUDE_DIRS` pointing to `${INSALL_DIR}/include/DLoopDetector`, typically it's `C:/Program Files/DLoopDetector/include/DLoopDetector`

Install Okvis+:

1.    Clone source from https://github.com/joemenke/okvis/. Cd to the directory. 

2.    Follow CMAKE steps above. 

Install Open3D:

1. Call git clone --recursive on the repo from https://github.com/intel-isl/Open3D. Extract the zip folder and cd to the extracted directory.

2. Follow CMAKE steps above, ADD “–parallel <number of cores>” to step 2 of the CMAKE build instructions. This will improve Open3D’s performance.
 

**Setting up the Intel Realsense D435i:**                                        

1.    Follow the instructions here: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_windows.md to prepare to install librealsense. Be sure to follow the “Enabling metadata on Windows” section as the timestamps coming from the device will be incorrect without it. 

2. Clone source from https://github.com/IntelRealSense/librealsense and `git checkout ba7c2d9cd59fb1618b9dc634cff7fa6349ce8bad`. Cd to the directory.

3. Follow CMAKE steps above.

4. You will need to add the environment variable `realsense2_DIR` and set it to the install location of realsense2 (probably C:\Program Files\librealsense\lib\cmake\realsense2) 

5.  You will also need to add the bin directory to the PATH variable. Select the variable “Path” and add a line with the value `C:\Program Files\librealsense2\bin`

5.    Update the device firmware using the windows firmware update tool: https://downloadcenter.intel.com/download/27514/Windows-Device-Firmware-Update-Tool-for-Intel-RealSense-D400-Product-Family

6.    Using the “realsense-viewer” tool (located in build/tools/realsense-viewer), ensure that all sensors are able to stream data.


**Building OpenARK:**

1.    Open an Administrator Command Prompt or a VS2015 x64 Native Tools Command Prompt


2.    Cd to the directory to which you have downloaded the OpenARK source code


3.    Make a build directory to store build files: 
    `mkdir build`


4.    Generate a Visual Studio solution: 
    
    `cd build`
    `cmake -G"Visual Studio 14 2015 Win64" ..`
    


5.    You can now either open the Visual Studio Solution generated in the build directory labeled “OpenARK.sln” or continue to build using the command prompt.


**Building via Visual Studio:**


 Right click on the OpenARK project in the solution explorer and select “Set as Startup Project”. Build and run as usual.

**Building via Command Prompt:** 

    `cmake --build . --config Release`

 **To run the OpenARK hand demo:**
    `cd Release`
    `OpenARK_hand_demo.exe`

**To run the OpenARK SLAM demo:**

    `cd Release`
    `OpenARK_SLAM_demo.exe`

**Add these lines to your intrinsics yaml file**

    ```
    numKeypointsResetThreshold: 10
    durationResetThreshold: 0.5
    emitterPower: 0.
    ```

**General Debugging Strategies:**

**Types of Potential Errors:**

1. Unable to find a file/path/directory for a previously installed package/dependency:

It is likely that one of your environment variables has been set incorrectly.

a. Check environment variables and make sure that the variables are specified to the correct directories in accordance with the instructions. If you need to make any changes, make sure to close any current session of command prompt and start up a new one after modifying environment variables.

b. If this still doesn't work, there is a manual workaround: open CMakeList.txt, navigate to the erroring lines specified in the stack trace, and replace the automatic script for locating the directory with the manual path directory itself.

2. Error for a package relates to an earlier package/dependency from previous steps in the installation process:

It is likely that the package/dependency has not been installed correctly earlier in the installation process.

a. Locate erroring earlier package and delete entire build folder. Rerun CMake steps on this earlier pacakge.

b. Navigate back to current package. Delete CMakeCache.txt in build folder. Rerurn CMake on this current package.

3. Other errors:

a. Check on stackoverflow/other online resources for help.

b. Open an issue and ask us for help!
