cmake_minimum_required(VERSION 3.10)
project(OpenCVTest)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Demo demo.cpp)
target_link_libraries(Demo ${OpenCV_LIBRARIES})
