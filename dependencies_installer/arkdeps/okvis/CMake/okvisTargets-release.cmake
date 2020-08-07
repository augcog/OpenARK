#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "okvis_util" for configuration "Release"
set_property(TARGET okvis_util APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_util PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_util.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_util )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_util "C:/Program Files/okvis/lib/okvis_util.lib" )

# Import target "okvis_kinematics" for configuration "Release"
set_property(TARGET okvis_kinematics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_kinematics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_util"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_kinematics.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_kinematics )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_kinematics "C:/Program Files/okvis/lib/okvis_kinematics.lib" )

# Import target "okvis_time" for configuration "Release"
set_property(TARGET okvis_time APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_time PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_time.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_time )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_time "C:/Program Files/okvis/lib/okvis_time.lib" )

# Import target "okvis_cv" for configuration "Release"
set_property(TARGET okvis_cv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_cv PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_util;okvis_kinematics;okvis_time;opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab;opencv_world;opencv_aruco;opencv_bgsegm;opencv_ccalib;opencv_datasets;opencv_dnn_objdetect;opencv_dpm;opencv_face;opencv_img_hash;opencv_plot;opencv_rgbd;opencv_stereo;opencv_surface_matching;opencv_tracking;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto;C:/Program Files/OpenARK-Deps/brisk/lib/brisk.lib;C:/Program Files/OpenARK-Deps/brisk/lib/agast.lib"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_cv.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_cv )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_cv "C:/Program Files/okvis/lib/okvis_cv.lib" )

# Import target "okvis_common" for configuration "Release"
set_property(TARGET okvis_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_common PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_util;okvis_kinematics;okvis_time;okvis_cv"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_common.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_common )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_common "C:/Program Files/okvis/lib/okvis_common.lib" )

# Import target "okvis_ceres" for configuration "Release"
set_property(TARGET okvis_ceres APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_ceres PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_util;okvis_cv;okvis_common;ceres"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_ceres.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_ceres )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_ceres "C:/Program Files/okvis/lib/okvis_ceres.lib" )

# Import target "okvis_timing" for configuration "Release"
set_property(TARGET okvis_timing APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_timing PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_util"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_timing.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_timing )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_timing "C:/Program Files/okvis/lib/okvis_timing.lib" )

# Import target "okvis_matcher" for configuration "Release"
set_property(TARGET okvis_matcher APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_matcher PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_util"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_matcher.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_matcher )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_matcher "C:/Program Files/okvis/lib/okvis_matcher.lib" )

# Import target "okvis_frontend" for configuration "Release"
set_property(TARGET okvis_frontend APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_frontend PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "C:/Program Files/OpenARK-Deps/brisk/lib/brisk.lib;C:/Program Files/OpenARK-Deps/brisk/lib/agast.lib;C:/Program Files/opengv/lib/opengv.lib;ceres;Boost::date_time;Boost::filesystem;Boost::system;okvis_util;okvis_cv;okvis_ceres;okvis_timing;okvis_matcher"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_frontend.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_frontend )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_frontend "C:/Program Files/okvis/lib/okvis_frontend.lib" )

# Import target "okvis_multisensor_processing" for configuration "Release"
set_property(TARGET okvis_multisensor_processing APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(okvis_multisensor_processing PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "okvis_time;okvis_util;okvis_kinematics;okvis_cv;okvis_common;okvis_ceres;okvis_frontend;glog::glog"
  IMPORTED_LOCATION_RELEASE "C:/Program Files/okvis/lib/okvis_multisensor_processing.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS okvis_multisensor_processing )
list(APPEND _IMPORT_CHECK_FILES_FOR_okvis_multisensor_processing "C:/Program Files/okvis/lib/okvis_multisensor_processing.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
