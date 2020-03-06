cmake_minimum_required(VERSION 3.5)
project(catchit)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp camera.h)
add_executable(catchit ${SOURCE_FILES})

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(catchit ${OpenCV_LIBS})