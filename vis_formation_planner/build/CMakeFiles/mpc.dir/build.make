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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/weijian/Heterogeneous_formation/src/vis_formation_planner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weijian/Heterogeneous_formation/src/vis_formation_planner/build

# Include any dependencies generated for this target.
include CMakeFiles/mpc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mpc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mpc.dir/flags.make

CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.o: CMakeFiles/mpc.dir/flags.make
CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.o: ../src/trajectory_tracking/mpc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weijian/Heterogeneous_formation/src/vis_formation_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.o -c /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/mpc.cpp

CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/mpc.cpp > CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.i

CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/mpc.cpp -o CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.s

CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.o: CMakeFiles/mpc.dir/flags.make
CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.o: ../src/trajectory_tracking/vec2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weijian/Heterogeneous_formation/src/vis_formation_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.o -c /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/vec2d.cpp

CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/vec2d.cpp > CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.i

CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/vec2d.cpp -o CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.s

CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.o: CMakeFiles/mpc.dir/flags.make
CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.o: ../src/trajectory_tracking/plot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weijian/Heterogeneous_formation/src/vis_formation_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.o -c /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/plot.cpp

CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/plot.cpp > CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.i

CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weijian/Heterogeneous_formation/src/vis_formation_planner/src/trajectory_tracking/plot.cpp -o CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.s

# Object files for target mpc
mpc_OBJECTS = \
"CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.o" \
"CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.o" \
"CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.o"

# External object files for target mpc
mpc_EXTERNAL_OBJECTS =

devel/lib/vis_formation_planner/mpc: CMakeFiles/mpc.dir/src/trajectory_tracking/mpc.cpp.o
devel/lib/vis_formation_planner/mpc: CMakeFiles/mpc.dir/src/trajectory_tracking/vec2d.cpp.o
devel/lib/vis_formation_planner/mpc: CMakeFiles/mpc.dir/src/trajectory_tracking/plot.cpp.o
devel/lib/vis_formation_planner/mpc: CMakeFiles/mpc.dir/build.make
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libdynamicedt3d.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libbase_local_planner.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libtrajectory_planner_ros.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libgazebo_ros_api_plugin.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libgazebo_ros_paths_plugin.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libcostmap_2d.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/liblayers.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/liblaser_geometry.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libvoxel_grid.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libclass_loader.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libroslib.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/librospack.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libpython3.8.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libinteractive_markers.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/liboctomap_ros.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/liboctomap.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/liboctomath.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libtf.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libtf2_ros.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libactionlib.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libmessage_filters.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libroscpp.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libtf2.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/librosconsole.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/librostime.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
devel/lib/vis_formation_planner/mpc: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
devel/lib/vis_formation_planner/mpc: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/vis_formation_planner/mpc: CMakeFiles/mpc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/weijian/Heterogeneous_formation/src/vis_formation_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable devel/lib/vis_formation_planner/mpc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mpc.dir/build: devel/lib/vis_formation_planner/mpc

.PHONY : CMakeFiles/mpc.dir/build

CMakeFiles/mpc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mpc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mpc.dir/clean

CMakeFiles/mpc.dir/depend:
	cd /home/weijian/Heterogeneous_formation/src/vis_formation_planner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weijian/Heterogeneous_formation/src/vis_formation_planner /home/weijian/Heterogeneous_formation/src/vis_formation_planner /home/weijian/Heterogeneous_formation/src/vis_formation_planner/build /home/weijian/Heterogeneous_formation/src/vis_formation_planner/build /home/weijian/Heterogeneous_formation/src/vis_formation_planner/build/CMakeFiles/mpc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mpc.dir/depend

