# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/cmake/252/bin/cmake

# The command to remove a file.
RM = /snap/cmake/252/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/drew/Documents/Drew/Drone/catchIt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/catchIt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/catchIt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/catchIt.dir/flags.make

CMakeFiles/catchIt.dir/main.cpp.o: CMakeFiles/catchIt.dir/flags.make
CMakeFiles/catchIt.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/catchIt.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/catchIt.dir/main.cpp.o -c /home/drew/Documents/Drew/Drone/catchIt/main.cpp

CMakeFiles/catchIt.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/catchIt.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/drew/Documents/Drew/Drone/catchIt/main.cpp > CMakeFiles/catchIt.dir/main.cpp.i

CMakeFiles/catchIt.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/catchIt.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/drew/Documents/Drew/Drone/catchIt/main.cpp -o CMakeFiles/catchIt.dir/main.cpp.s

# Object files for target catchIt
catchIt_OBJECTS = \
"CMakeFiles/catchIt.dir/main.cpp.o"

# External object files for target catchIt
catchIt_EXTERNAL_OBJECTS =

catchIt: CMakeFiles/catchIt.dir/main.cpp.o
catchIt: CMakeFiles/catchIt.dir/build.make
catchIt: CMakeFiles/catchIt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable catchIt"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/catchIt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/catchIt.dir/build: catchIt

.PHONY : CMakeFiles/catchIt.dir/build

CMakeFiles/catchIt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/catchIt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/catchIt.dir/clean

CMakeFiles/catchIt.dir/depend:
	cd /home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/drew/Documents/Drew/Drone/catchIt /home/drew/Documents/Drew/Drone/catchIt /home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug /home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug /home/drew/Documents/Drew/Drone/catchIt/cmake-build-debug/CMakeFiles/catchIt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/catchIt.dir/depend

