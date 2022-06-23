cmake_minimum_required(VERSION 3.22)
project(engineering_project)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_STANDARD 20)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(engineering_project main.cpp PclUtils.cpp PclUtils.h DisparityMap.cpp DisparityMap.h)
target_link_libraries (engineering_project ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

