# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/julia/cmake-3.23.5/bin/cmake

# The command to remove a file.
RM = /home/julia/cmake-3.23.5/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/julia/catkin_ws/build/rotation_estimator

# Include any dependencies generated for this target.
include CMakeFiles/rotation_estimation.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/rotation_estimation.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/rotation_estimation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rotation_estimation.dir/flags.make

CMakeFiles/rotation_estimation.dir/src/main.cpp.o: CMakeFiles/rotation_estimation.dir/flags.make
CMakeFiles/rotation_estimation.dir/src/main.cpp.o: /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator/src/main.cpp
CMakeFiles/rotation_estimation.dir/src/main.cpp.o: CMakeFiles/rotation_estimation.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rotation_estimation.dir/src/main.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimation.dir/src/main.cpp.o -MF CMakeFiles/rotation_estimation.dir/src/main.cpp.o.d -o CMakeFiles/rotation_estimation.dir/src/main.cpp.o -c /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator/src/main.cpp

CMakeFiles/rotation_estimation.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimation.dir/src/main.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator/src/main.cpp > CMakeFiles/rotation_estimation.dir/src/main.cpp.i

CMakeFiles/rotation_estimation.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimation.dir/src/main.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator/src/main.cpp -o CMakeFiles/rotation_estimation.dir/src/main.cpp.s

# Object files for target rotation_estimation
rotation_estimation_OBJECTS = \
"CMakeFiles/rotation_estimation.dir/src/main.cpp.o"

# External object files for target rotation_estimation
rotation_estimation_EXTERNAL_OBJECTS =

/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: CMakeFiles/rotation_estimation.dir/src/main.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: CMakeFiles/rotation_estimation.dir/build.make
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /home/julia/catkin_ws/devel/.private/rotation_estimator/lib/librotation_estimator.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_dnn.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_highgui.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_ml.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_objdetect.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_shape.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_stitching.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_superres.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_videostab.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_viz.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libcv_bridge.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libeigen_conversions.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libmessage_filters.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libroscpp.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/librosconsole.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/librostime.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /opt/ros/melodic/lib/libcpp_common.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_calib3d.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_features2d.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_flann.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_photo.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_video.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_videoio.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_imgcodecs.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_imgproc.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libopencv_core.so.3.4.8
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: /usr/local/lib/libfmt.a
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation: CMakeFiles/rotation_estimation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rotation_estimation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rotation_estimation.dir/build: /home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimation
.PHONY : CMakeFiles/rotation_estimation.dir/build

CMakeFiles/rotation_estimation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rotation_estimation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rotation_estimation.dir/clean

CMakeFiles/rotation_estimation.dir/depend:
	cd /home/julia/catkin_ws/build/rotation_estimator && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator /home/julia/catkin_ws/src/VAVM-A/src/rotation_estimator /home/julia/catkin_ws/build/rotation_estimator /home/julia/catkin_ws/build/rotation_estimator /home/julia/catkin_ws/build/rotation_estimator/CMakeFiles/rotation_estimation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rotation_estimation.dir/depend

