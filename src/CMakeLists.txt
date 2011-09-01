cmake_minimum_required( VERSION 2.8 )

# This CMakeLists is assumed to be loaded from the parent CMakeLists and inherits
# all the CUDA variables

ADD_DEFINITIONS( -fPIC )

set(CPACK_PACKAGE_NAME python-turbopixels)
set(CPACK_PACKAGE_VERSION 3.0)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE amd64)
set(CPACK_PACKAGE_CONTACT "amueller@ais.uni-bonn.de")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Turbopixel using CUDA by Alvaro Collet and Alex Rodionov")
INCLUDE(CPack)

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

FIND_PACKAGE( Boost 1.37 COMPONENTS python REQUIRED )
FIND_PACKAGE( PythonLibs )
PYTHON_ADD_MODULE(turbopixel_python SHARED turbopixel_python.cpp)

target_link_libraries(turbopixel_python
    ${Boost_PYTHON_LIBRARY_RELEASE}
    ${PYTHON_LIBRARY}
    turbopix
    )
