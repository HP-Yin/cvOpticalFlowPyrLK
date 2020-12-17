cmake_minimum_required(VERSION 2.8)
project(OpticalFlowPyrLK)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

find_package(OpenCV 3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
)

add_library(OpticalFlowPyrLK SHARED
    OpticalFlowPyrLK.cpp
)

add_executable(TestOpticalFlow TestOpticalFlow.cpp)
target_link_libraries(TestOpticalFlow ${OpenCV_LIBS} OpticalFlowPyrLK)

