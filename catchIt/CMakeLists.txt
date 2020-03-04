cmake_minimum_required(VERSION 3.15)
project(catchIt)

set(CMAKE_CXX_STANDARD 14)

add_executable(catchIt main.cpp camera.h)

find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# Specify at least C++11
add_definitions("-std=c++11")

# Enable strict handling of warnings
add_definitions("-Wall -Wextra")

# In case /usr/local/include is not in your default includes
#include_directories(/usr/local/include)

# In case /usr/local/lib is not in your default linker path
#link_directories(/usr/local/lib)

# Specify your app's executable name, and list of source files used to create it.
add_executable(catchItExecutable
        main.cpp
        CalculatePosition.h
        camera.h
        )

# Specify your app's executable name and a list of linked libraries
target_link_libraries(catchItExecutable
        mavsdk          #All apps link against mavsdk library
        mavsdk_action      # If action plugin used by app ...
        mavsdk_telemetry   #If telemetry plugin used by app ...
        mavsdk_offboard
        # ... Any other linked libraries
        )