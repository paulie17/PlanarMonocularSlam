cmake_minimum_required(VERSION 2.8)

#ds project name
project(planar_monocular_slam)

#ds set cmake build flags
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3 -DNDEBUG") #ds enable all warnings, c++11 features, enable maximum optimization and disable debug symbols

list(APPEND CMAKE_MODULE_PATH ${planar_monocular_slam_SOURCE_DIR}/cmake_modules)

#ds find and set up dependencies

#find_package(g2o REQUIRED)
#include_directories(${g2o_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(SuiteSparse REQUIRED)
include_directories(${SuiteSparse_INCLUDE_DIRS})

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/gnuplot-iostream")

#ds set internal includes (our files)
include_directories(src)

#ds add build targets
add_subdirectory(executables)


