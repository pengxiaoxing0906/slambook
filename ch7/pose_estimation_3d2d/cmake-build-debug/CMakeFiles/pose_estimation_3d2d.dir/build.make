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
CMAKE_SOURCE_DIR = /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/pose_estimation_3d2d.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pose_estimation_3d2d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pose_estimation_3d2d.dir/flags.make

CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.o: CMakeFiles/pose_estimation_3d2d.dir/flags.make
CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.o: ../main1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.o -c /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/main1.cpp

CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/main1.cpp > CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.i

CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/main1.cpp -o CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.s

# Object files for target pose_estimation_3d2d
pose_estimation_3d2d_OBJECTS = \
"CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.o"

# External object files for target pose_estimation_3d2d
pose_estimation_3d2d_EXTERNAL_OBJECTS =

pose_estimation_3d2d: CMakeFiles/pose_estimation_3d2d.dir/main1.cpp.o
pose_estimation_3d2d: CMakeFiles/pose_estimation_3d2d.dir/build.make
pose_estimation_3d2d: /usr/local/lib/libopencv_viz.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_videostab.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_shape.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_ml.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_photo.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_stitching.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_dnn.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_objdetect.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_superres.so.3.4.1
pose_estimation_3d2d: /usr/lib/x86_64-linux-gnu/libcxsparse.so
pose_estimation_3d2d: /usr/local/lib/libopencv_calib3d.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_features2d.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_highgui.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_flann.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_video.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_videoio.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_imgproc.so.3.4.1
pose_estimation_3d2d: /usr/local/lib/libopencv_core.so.3.4.1
pose_estimation_3d2d: CMakeFiles/pose_estimation_3d2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pose_estimation_3d2d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pose_estimation_3d2d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pose_estimation_3d2d.dir/build: pose_estimation_3d2d

.PHONY : CMakeFiles/pose_estimation_3d2d.dir/build

CMakeFiles/pose_estimation_3d2d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pose_estimation_3d2d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pose_estimation_3d2d.dir/clean

CMakeFiles/pose_estimation_3d2d.dir/depend:
	cd /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug /home/pxx/Documents/slambook/ch7/pose_estimation_3d2d/cmake-build-debug/CMakeFiles/pose_estimation_3d2d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pose_estimation_3d2d.dir/depend

