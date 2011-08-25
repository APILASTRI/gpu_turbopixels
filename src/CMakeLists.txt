cmake_minimum_required( VERSION 2.8 )

# This CMakeLists is assumed to be loaded from the parent CMakeLists and inherits
# all the CUDA variables and ROS stuff

ADD_DEFINITIONS( -fPIC )


include_directories ( ${PROJECT_SOURCE_DIR}/include )
include_directories ( ${PROJECT_SOURCE_DIR}/src )
include_directories ( ${CUDA_SDK_ROOT_DIR}/common/inc )

cuda_add_library( turbopix SHARED Turbopixels.cpp turbopix.cu reduction.cu recursiveGaussian.cu )
target_link_libraries ( turbopix )

###############################################################################
################ CUSTOM LINKING: OpenCV (C++) + CUDA ##########################
###############################################################################
# We don't need opencv to create the library, but we do for the executable
# We need custom linking for opencv for now, because we use cuda_add_executable

include_directories (/usr/include/opencv-2.3.1/)
#link_directories ( /usr/lib/opencv-2.3.1/)
cuda_add_executable( example_tpix example.cpp )
#target_link_libraries ( example_tpix ${CVLIB} ${CUDA_CUT_LIBRARY} turbopix )
#target_link_libraries ( example_tpix ${CVLIB} turbopix )
target_link_libraries ( example_tpix opencv_core opencv_highgui turbopix )

#target_link_libraries ( turbopix cv ${CUDA_CUT_LIBRARY})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(extract_3d_features src/extract_3d_features.cpp)
#target_link_libraries(example ${PROJECT_NAME})
