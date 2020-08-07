# https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file
# https://cmake.org/cmake/help/v3.0/command/find_package.html?highlight=find_package
# https://cmake.org/cmake/help/v3.0/module/CMakePackageConfigHelpers.html
# Config file for the Open3D package
# It defines the following variables
#  Open3D_INCLUDE_DIRS
#  Open3D_LIBRARIES
#  Open3D_LIBRARY_DIRS


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Open3DConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)

####################################################################################

set(Open3D_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/Eigen;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/glew/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/GLFW/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/fmt/include")
set(Open3D_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib")
set(Open3D_LIBRARIES    "Open3D;glew;glfw3;opengl32;turbojpeg-static;jsoncpp;png;zlib;tinyfiledialogs;tinyobjloader;qhullcpp;qhullstatic_r")

set(Open3D_C_FLAGS          "-openmp")
set(Open3D_CXX_FLAGS        "-openmp")
set(Open3D_EXE_LINKER_FLAGS "")
