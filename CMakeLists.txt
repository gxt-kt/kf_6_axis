cmake_minimum_required(VERSION 3.00)
project(kf_6_axis)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

# find_package(
#   EIGEN3
#   # OpenCV REQUIRED
#   # Boost REQUIRED
#   # OpenCV REQUIRED
#   )
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(kf)

# message(STATUS "opencv_inc=" ${OpenCV_INCLUDE_DIRS})
# message(STATUS "opencv_lib=" ${OpenCV_LIBRARIES})

#find_library(Boost 1.80.0)
# set(CMAKE_BUILD_TYPE RelWithDebInfo) # 使能优化的gdb调试
# add_executable(untitled main.cpp test.cpp ws2812.cpp )
add_executable(untitled main.cpp readdata.hpp )
