# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/akeaveny/cpp/roboteq_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/akeaveny/cpp/roboteq_driver

# Include any dependencies generated for this target.
include CMakeFiles/roboteq_driver_example_main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/roboteq_driver_example_main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/roboteq_driver_example_main.dir/flags.make

CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o: CMakeFiles/roboteq_driver_example_main.dir/flags.make
CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o: src/example_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akeaveny/cpp/roboteq_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o -c /home/akeaveny/cpp/roboteq_driver/src/example_main.cpp

CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/akeaveny/cpp/roboteq_driver/src/example_main.cpp > CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.i

CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/akeaveny/cpp/roboteq_driver/src/example_main.cpp -o CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.s

CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.requires:

.PHONY : CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.requires

CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.provides: CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.requires
	$(MAKE) -f CMakeFiles/roboteq_driver_example_main.dir/build.make CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.provides.build
.PHONY : CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.provides

CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.provides.build: CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o


# Object files for target roboteq_driver_example_main
roboteq_driver_example_main_OBJECTS = \
"CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o"

# External object files for target roboteq_driver_example_main
roboteq_driver_example_main_EXTERNAL_OBJECTS =

roboteq_driver_example_main: CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o
roboteq_driver_example_main: CMakeFiles/roboteq_driver_example_main.dir/build.make
roboteq_driver_example_main: libroboteq_driver.a
roboteq_driver_example_main: CMakeFiles/roboteq_driver_example_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/akeaveny/cpp/roboteq_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable roboteq_driver_example_main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/roboteq_driver_example_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/roboteq_driver_example_main.dir/build: roboteq_driver_example_main

.PHONY : CMakeFiles/roboteq_driver_example_main.dir/build

CMakeFiles/roboteq_driver_example_main.dir/requires: CMakeFiles/roboteq_driver_example_main.dir/src/example_main.cpp.o.requires

.PHONY : CMakeFiles/roboteq_driver_example_main.dir/requires

CMakeFiles/roboteq_driver_example_main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/roboteq_driver_example_main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/roboteq_driver_example_main.dir/clean

CMakeFiles/roboteq_driver_example_main.dir/depend:
	cd /home/akeaveny/cpp/roboteq_driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/akeaveny/cpp/roboteq_driver /home/akeaveny/cpp/roboteq_driver /home/akeaveny/cpp/roboteq_driver /home/akeaveny/cpp/roboteq_driver /home/akeaveny/cpp/roboteq_driver/CMakeFiles/roboteq_driver_example_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/roboteq_driver_example_main.dir/depend
