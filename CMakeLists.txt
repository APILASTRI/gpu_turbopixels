cmake_minimum_required( VERSION 2.8 )

SET(CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules )

project ( turbopix )

FIND_PACKAGE(CUDA)
#FIND_PACKAGE(OpenCV)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

subdirs ( ${PROJECT_SOURCE_DIR}/src )


