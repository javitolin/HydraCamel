# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build

# Include any dependencies generated for this target.
include camera_input/CMakeFiles/camera_driver_run.dir/depend.make

# Include the progress variables for this target.
include camera_input/CMakeFiles/camera_driver_run.dir/progress.make

# Include the compile flags for this target's objects.
include camera_input/CMakeFiles/camera_driver_run.dir/flags.make

camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o: camera_input/CMakeFiles/camera_driver_run.dir/flags.make
camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o: /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src/camera_input/src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o"
	cd /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camera_driver_run.dir/src/main.cpp.o -c /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src/camera_input/src/main.cpp

camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_driver_run.dir/src/main.cpp.i"
	cd /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src/camera_input/src/main.cpp > CMakeFiles/camera_driver_run.dir/src/main.cpp.i

camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_driver_run.dir/src/main.cpp.s"
	cd /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src/camera_input/src/main.cpp -o CMakeFiles/camera_driver_run.dir/src/main.cpp.s

camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.requires:
.PHONY : camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.requires

camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.provides: camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.requires
	$(MAKE) -f camera_input/CMakeFiles/camera_driver_run.dir/build.make camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.provides.build
.PHONY : camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.provides

camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.provides.build: camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o

# Object files for target camera_driver_run
camera_driver_run_OBJECTS = \
"CMakeFiles/camera_driver_run.dir/src/main.cpp.o"

# External object files for target camera_driver_run
camera_driver_run_EXTERNAL_OBJECTS =

/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: camera_input/CMakeFiles/camera_driver_run.dir/build.make
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libcv_bridge.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libimage_transport.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libmessage_filters.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libclass_loader.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/libPocoFoundation.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libdl.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libroscpp.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/librosconsole.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/liblog4cxx.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libroslib.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/librostime.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /opt/ros/indigo/lib/libcpp_common.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/local/lib/libopencv_core.so.2.4.9
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/local/lib/libopencv_highgui.so.2.4.9
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/local/lib/libopencv_imgproc.so.2.4.9
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: /usr/local/lib/libopencv_core.so.2.4.9
/home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run: camera_input/CMakeFiles/camera_driver_run.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run"
	cd /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_driver_run.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
camera_input/CMakeFiles/camera_driver_run.dir/build: /home/jdorfsman/git/HydraCamel/ROS_PROJECT/devel/lib/camera_input/camera_driver_run
.PHONY : camera_input/CMakeFiles/camera_driver_run.dir/build

camera_input/CMakeFiles/camera_driver_run.dir/requires: camera_input/CMakeFiles/camera_driver_run.dir/src/main.cpp.o.requires
.PHONY : camera_input/CMakeFiles/camera_driver_run.dir/requires

camera_input/CMakeFiles/camera_driver_run.dir/clean:
	cd /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input && $(CMAKE_COMMAND) -P CMakeFiles/camera_driver_run.dir/cmake_clean.cmake
.PHONY : camera_input/CMakeFiles/camera_driver_run.dir/clean

camera_input/CMakeFiles/camera_driver_run.dir/depend:
	cd /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src /home/jdorfsman/git/HydraCamel/ROS_PROJECT/src/camera_input /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input /home/jdorfsman/git/HydraCamel/ROS_PROJECT/build/camera_input/CMakeFiles/camera_driver_run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : camera_input/CMakeFiles/camera_driver_run.dir/depend
