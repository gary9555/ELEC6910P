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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build

# Include any dependencies generated for this target.
include tag_detector/CMakeFiles/tag_detector.dir/depend.make

# Include the progress variables for this target.
include tag_detector/CMakeFiles/tag_detector.dir/progress.make

# Include the compile flags for this target's objects.
include tag_detector/CMakeFiles/tag_detector.dir/flags.make

tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o: tag_detector/CMakeFiles/tag_detector.dir/flags.make
tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o: /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src/tag_detector/src/tag_detector_node.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o"
	cd /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o -c /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src/tag_detector/src/tag_detector_node.cpp

tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.i"
	cd /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src/tag_detector/src/tag_detector_node.cpp > CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.i

tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.s"
	cd /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src/tag_detector/src/tag_detector_node.cpp -o CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.s

tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.requires:
.PHONY : tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.requires

tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.provides: tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.requires
	$(MAKE) -f tag_detector/CMakeFiles/tag_detector.dir/build.make tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.provides.build
.PHONY : tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.provides

tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.provides.build: tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o

# Object files for target tag_detector
tag_detector_OBJECTS = \
"CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o"

# External object files for target tag_detector
tag_detector_EXTERNAL_OBJECTS =

/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: tag_detector/CMakeFiles/tag_detector.dir/build.make
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/libroscpp.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/libcv_bridge.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/librosconsole.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/liblog4cxx.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/librostime.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /opt/ros/indigo/lib/libcpp_common.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_videostab.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_video.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_superres.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_stitching.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_photo.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ocl.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_objdetect.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_nonfree.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ml.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_legacy.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_imgproc.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_highgui.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_gpu.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_flann.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_features2d.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_core.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_contrib.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_calib3d.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_videostab.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_video.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ts.a
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_superres.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_stitching.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_photo.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ocl.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_objdetect.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_nonfree.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ml.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_legacy.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_imgproc.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_highgui.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_gpu.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_flann.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_features2d.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_core.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_contrib.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_calib3d.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_nonfree.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ocl.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_gpu.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_photo.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_objdetect.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_legacy.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_ml.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_video.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_calib3d.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_features2d.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_highgui.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_imgproc.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_flann.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: /usr/local/lib/libopencv_core.so.2.4.13
/home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector: tag_detector/CMakeFiles/tag_detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector"
	cd /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tag_detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tag_detector/CMakeFiles/tag_detector.dir/build: /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/devel/lib/tag_detector/tag_detector
.PHONY : tag_detector/CMakeFiles/tag_detector.dir/build

tag_detector/CMakeFiles/tag_detector.dir/requires: tag_detector/CMakeFiles/tag_detector.dir/src/tag_detector_node.cpp.o.requires
.PHONY : tag_detector/CMakeFiles/tag_detector.dir/requires

tag_detector/CMakeFiles/tag_detector.dir/clean:
	cd /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector && $(CMAKE_COMMAND) -P CMakeFiles/tag_detector.dir/cmake_clean.cmake
.PHONY : tag_detector/CMakeFiles/tag_detector.dir/clean

tag_detector/CMakeFiles/tag_detector.dir/depend:
	cd /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/src/tag_detector /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector /home/gary9555/Desktop/ELEC6910P/proj2phase2/catkin_ws/build/tag_detector/CMakeFiles/tag_detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tag_detector/CMakeFiles/tag_detector.dir/depend

