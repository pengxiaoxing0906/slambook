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
CMAKE_SOURCE_DIR = /home/pxx/Documents/slambook/ch10/ceres_custombundle

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/ceres_customBundle.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ceres_customBundle.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ceres_customBundle.dir/flags.make

CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.o: CMakeFiles/ceres_customBundle.dir/flags.make
CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.o: ../ceresBundle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.o -c /home/pxx/Documents/slambook/ch10/ceres_custombundle/ceresBundle.cpp

CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pxx/Documents/slambook/ch10/ceres_custombundle/ceresBundle.cpp > CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.i

CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pxx/Documents/slambook/ch10/ceres_custombundle/ceresBundle.cpp -o CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.s

# Object files for target ceres_customBundle
ceres_customBundle_OBJECTS = \
"CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.o"

# External object files for target ceres_customBundle
ceres_customBundle_EXTERNAL_OBJECTS =

ceres_customBundle: CMakeFiles/ceres_customBundle.dir/ceresBundle.cpp.o
ceres_customBundle: CMakeFiles/ceres_customBundle.dir/build.make
ceres_customBundle: libBALProblem.so
ceres_customBundle: libParseCmd.so
ceres_customBundle: /usr/local/lib/libceres.a
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libglog.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libgflags.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libspqr.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libtbb.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libcholmod.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libccolamd.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libcamd.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libcolamd.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libamd.so
ceres_customBundle: /usr/lib/liblapack.so
ceres_customBundle: /usr/lib/libblas.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/librt.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libcxsparse.so
ceres_customBundle: /usr/lib/liblapack.so
ceres_customBundle: /usr/lib/libblas.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/librt.so
ceres_customBundle: /usr/lib/x86_64-linux-gnu/libcxsparse.so
ceres_customBundle: CMakeFiles/ceres_customBundle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ceres_customBundle"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ceres_customBundle.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ceres_customBundle.dir/build: ceres_customBundle

.PHONY : CMakeFiles/ceres_customBundle.dir/build

CMakeFiles/ceres_customBundle.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ceres_customBundle.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ceres_customBundle.dir/clean

CMakeFiles/ceres_customBundle.dir/depend:
	cd /home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pxx/Documents/slambook/ch10/ceres_custombundle /home/pxx/Documents/slambook/ch10/ceres_custombundle /home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug /home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug /home/pxx/Documents/slambook/ch10/ceres_custombundle/cmake-build-debug/CMakeFiles/ceres_customBundle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ceres_customBundle.dir/depend

