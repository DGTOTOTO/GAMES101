cmake_minimum_required(VERSION 2.8.11)
project(homework0)

# 手动设置 Eigen3 的路径
set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(main main.cpp)