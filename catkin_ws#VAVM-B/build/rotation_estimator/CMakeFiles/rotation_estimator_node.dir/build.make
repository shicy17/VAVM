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
CMAKE_SOURCE_DIR = /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/julia/catkin_ws/build/rotation_estimator

# Include any dependencies generated for this target.
include CMakeFiles/rotation_estimator_node.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/rotation_estimator_node.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/rotation_estimator_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rotation_estimator_node.dir/flags.make

CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o: CMakeFiles/rotation_estimator_node.dir/flags.make
CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o: /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/rotation_estimator_node.cpp
CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o: CMakeFiles/rotation_estimator_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o -MF CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o.d -o CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o -c /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/rotation_estimator_node.cpp

CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/rotation_estimator_node.cpp > CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.i

CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/rotation_estimator_node.cpp -o CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.s

CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o: CMakeFiles/rotation_estimator_node.dir/flags.make
CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o: /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/database.cpp
CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o: CMakeFiles/rotation_estimator_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o -MF CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o.d -o CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o -c /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/database.cpp

CMakeFiles/rotation_estimator_node.dir/src/database.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimator_node.dir/src/database.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/database.cpp > CMakeFiles/rotation_estimator_node.dir/src/database.cpp.i

CMakeFiles/rotation_estimator_node.dir/src/database.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimator_node.dir/src/database.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/database.cpp -o CMakeFiles/rotation_estimator_node.dir/src/database.cpp.s

CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o: CMakeFiles/rotation_estimator_node.dir/flags.make
CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o: /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system.cpp
CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o: CMakeFiles/rotation_estimator_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o -MF CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o.d -o CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o -c /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system.cpp

CMakeFiles/rotation_estimator_node.dir/src/system.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimator_node.dir/src/system.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system.cpp > CMakeFiles/rotation_estimator_node.dir/src/system.cpp.i

CMakeFiles/rotation_estimator_node.dir/src/system.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimator_node.dir/src/system.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system.cpp -o CMakeFiles/rotation_estimator_node.dir/src/system.cpp.s

CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o: CMakeFiles/rotation_estimator_node.dir/flags.make
CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o: /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/numerics.cpp
CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o: CMakeFiles/rotation_estimator_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o -MF CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o.d -o CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o -c /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/numerics.cpp

CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/numerics.cpp > CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.i

CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/numerics.cpp -o CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.s

CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o: CMakeFiles/rotation_estimator_node.dir/flags.make
CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o: /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate.cpp
CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o: CMakeFiles/rotation_estimator_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o -MF CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o.d -o CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o -c /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate.cpp

CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate.cpp > CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.i

CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate.cpp -o CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.s

CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o: CMakeFiles/rotation_estimator_node.dir/flags.make
CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o: /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate_ransac_doublewarp_ceres.cpp
CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o: CMakeFiles/rotation_estimator_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o -MF CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o.d -o CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o -c /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate_ransac_doublewarp_ceres.cpp

CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.i"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate_ransac_doublewarp_ceres.cpp > CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.i

CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.s"
	/usr/bin/g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator/src/system_estimate_ransac_doublewarp_ceres.cpp -o CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.s

# Object files for target rotation_estimator_node
rotation_estimator_node_OBJECTS = \
"CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o" \
"CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o" \
"CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o" \
"CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o" \
"CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o" \
"CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o"

# External object files for target rotation_estimator_node
rotation_estimator_node_EXTERNAL_OBJECTS =

/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/src/rotation_estimator_node.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/src/database.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/src/system.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/src/numerics.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/src/system_estimate.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/src/system_estimate_ransac_doublewarp_ceres.cpp.o
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/build.make
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libcv_bridge.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libeigen_conversions.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libtf.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libtf2_ros.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libactionlib.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libtf2.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libmessage_filters.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libroscpp.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/librosconsole.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/librostime.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /opt/ros/melodic/lib/libcpp_common.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/local/lib/libceres.a
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5.2
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libglog.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libspqr.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libtbb.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcholmod.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libccolamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcolamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libf77blas.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libatlas.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/librt.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcxsparse.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libtbb.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcholmod.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libccolamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcolamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libamd.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libf77blas.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libatlas.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/librt.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: /usr/lib/x86_64-linux-gnu/libcxsparse.so
/home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node: CMakeFiles/rotation_estimator_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/julia/catkin_ws/build/rotation_estimator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable /home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rotation_estimator_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rotation_estimator_node.dir/build: /home/julia/catkin_ws/devel/.private/rotation_estimator/lib/rotation_estimator/rotation_estimator_node
.PHONY : CMakeFiles/rotation_estimator_node.dir/build

CMakeFiles/rotation_estimator_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rotation_estimator_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rotation_estimator_node.dir/clean

CMakeFiles/rotation_estimator_node.dir/depend:
	cd /home/julia/catkin_ws/build/rotation_estimator && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator /home/julia/catkin_ws/src/VAVM-B/src/rotation_estimator /home/julia/catkin_ws/build/rotation_estimator /home/julia/catkin_ws/build/rotation_estimator /home/julia/catkin_ws/build/rotation_estimator/CMakeFiles/rotation_estimator_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rotation_estimator_node.dir/depend

