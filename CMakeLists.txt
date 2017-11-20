cmake_minimum_required(VERSION 2.8)
project(visionlib)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(/usr/include/eigen3 ./common)

add_executable(test test.cpp common/vs_transform.cpp)