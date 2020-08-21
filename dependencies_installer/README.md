# OpenARK Dependency Pack Creator

Code for creating the [OpenARK](https://github.com/augcog/openark) dependency pack.

To create the installer:
1) Put all dependencies in `../arkdeps` directory (relative to the repo root)
2) Call python script to generate install_list, uninstall_list, and file_list logs
python gen_list_files_for_nsis.py "../arkdeps" install_list.nsh uninstall_list.nsh file_list.nsh
3) Use [NSIS](https://nsis.sourceforge.io) to compile `main.nsi`. The [EnVar](https://nsis.sourceforge.io/EnVar_plug-in) NSIS plugin is required.
`"C:\Program Files (x86)\NSIS\makensis.exe" /DINST_LIST=install_list.nsh /DUNINST_LIST=uninstall_list.nsh /DFILE_LIST=file_list.nsh main_log.nsi`

The created installer will appear in the creation folder.


### Contents
Due to size, PCL is not included. Please download it from:
https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2015-win64.exe

Make sure to select the option to add PCL to PATH. Boost and OpenNI will also be installed as part of this.

This dependency pack contains the following:
- Brisk
- Ceres
- DBoW2
- DLib
- DLoopDetector
- eigen3
- glog
- librealsense2
- okvis
- OpenCV + contrib
- OpenGV
- SuiteSparse
- Open3D

## Details

### binaries
 We expect the directory `../arkdeps/extra/bin` to contain all binaries (DLLs, executables) from dependencies. This will be added to PATH automatically after installation. Combining the binary directories helps to prevent PATH from getting too long.

### ARK_DEPS_DIR
 The ARK_DEPS_DIR environment variable will be set to the install directory. OpenARK's CMakeLists is set up to find the dependencies here if available.

### CMakeLists.txt
 The install directory should contain `CMakeLists.txt`, a CMake script to set dependency paths automatically.

### Uninstalling
 Due to a bug, the uninstall process is done by manually specifying all of the dependencies. If more dependencies are added in the future, they should also be specified in the uninstall section of main.nsi in order for the uninstaller to successfully  uninstall them.

## License
Apache 2.0. See [augcog/OpenARK](https://github.com/augcog/openark) for the license.
