# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/pxx/Downloads/clion-2018.2.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/pxx/Downloads/clion-2018.2.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pxx/Documents/slambook/ch3/RotationMatrix

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/exercise1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/exercise1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exercise1.dir/flags.make

CMakeFiles/exercise1.dir/main.cpp.o: CMakeFiles/exercise1.dir/flags.make
CMakeFiles/exercise1.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exercise1.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exercise1.dir/main.cpp.o -c /home/pxx/Documents/slambook/ch3/RotationMatrix/main.cpp

CMakeFiles/exercise1.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exercise1.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pxx/Documents/slambook/ch3/RotationMatrix/main.cpp > CMakeFiles/exercise1.dir/main.cpp.i

CMakeFiles/exercise1.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exercise1.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pxx/Documents/slambook/ch3/RotationMatrix/main.cpp -o CMakeFiles/exercise1.dir/main.cpp.s

# Object files for target exercise1
exercise1_OBJECTS = \
"CMakeFiles/exercise1.dir/main.cpp.o"

# External object files for target exercise1
exercise1_EXTERNAL_OBJECTS =

exercise1: CMakeFiles/exercise1.dir/main.cpp.o
exercise1: CMakeFiles/exercise1.dir/build.make
exercise1: CMakeFiles/exercise1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exercise1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exercise1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exercise1.dir/build: exercise1

.PHONY : CMakeFiles/exercise1.dir/build

CMakeFiles/exercise1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exercise1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exercise1.dir/clean

CMakeFiles/exercise1.dir/depend:
	cd /home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pxx/Documents/slambook/ch3/RotationMatrix /home/pxx/Documents/slambook/ch3/RotationMatrix /home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug /home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug /home/pxx/Documents/slambook/ch3/RotationMatrix/cmake-build-debug/CMakeFiles/exercise1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exercise1.dir/depend
