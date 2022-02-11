cmake_minimum_required(VERSION 2.8)

#ds project name
project(testing_the_dataset)

#ds set cmake build flags
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3 -DNDEBUG") #ds enable all warnings, c++11 features, enable maximum optimization and disable debug symbols

#ds find and set up dependencies

#find_package(g2o REQUIRED)
#include_directories(${g2o_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/gnuplot-iostream")

#ds set internal includes (our files)
include_directories(src)

#ds add build targets
add_subdirectory(executables)


