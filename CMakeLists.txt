cmake_minimum_required(VERSION 3.1)
project(lidar_slam_algorithm)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### Dependencies ####################
# PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common features visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV 3.4
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

############### source ######################
include_directories(${PROJECT_SOURCE_DIR}/inc)
add_subdirectory(utils)
add_subdirectory(src)