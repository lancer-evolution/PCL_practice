# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.1

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
CMAKE_SOURCE_DIR = /home/kazuhiro/PCL/tutorial/ICP-test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kazuhiro/PCL/tutorial/ICP-test/build

# Include any dependencies generated for this target.
include CMakeFiles/icp4_after_feature_registration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/icp4_after_feature_registration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/icp4_after_feature_registration.dir/flags.make

CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o: CMakeFiles/icp4_after_feature_registration.dir/flags.make
CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o: ../src/icp4_after_feature_registration.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/kazuhiro/PCL/tutorial/ICP-test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o -c /home/kazuhiro/PCL/tutorial/ICP-test/src/icp4_after_feature_registration.cpp

CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/kazuhiro/PCL/tutorial/ICP-test/src/icp4_after_feature_registration.cpp > CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.i

CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/kazuhiro/PCL/tutorial/ICP-test/src/icp4_after_feature_registration.cpp -o CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.s

CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.requires:
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.requires

CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.provides: CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp4_after_feature_registration.dir/build.make CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.provides.build
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.provides

CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.provides.build: CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o

CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o: CMakeFiles/icp4_after_feature_registration.dir/flags.make
CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o: ../src/visualize_correspondences.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/kazuhiro/PCL/tutorial/ICP-test/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o -c /home/kazuhiro/PCL/tutorial/ICP-test/src/visualize_correspondences.cpp

CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/kazuhiro/PCL/tutorial/ICP-test/src/visualize_correspondences.cpp > CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.i

CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/kazuhiro/PCL/tutorial/ICP-test/src/visualize_correspondences.cpp -o CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.s

CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.requires:
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.requires

CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.provides: CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp4_after_feature_registration.dir/build.make CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.provides.build
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.provides

CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.provides.build: CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o

# Object files for target icp4_after_feature_registration
icp4_after_feature_registration_OBJECTS = \
"CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o" \
"CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o"

# External object files for target icp4_after_feature_registration
icp4_after_feature_registration_EXTERNAL_OBJECTS =

icp4_after_feature_registration: CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o
icp4_after_feature_registration: CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o
icp4_after_feature_registration: CMakeFiles/icp4_after_feature_registration.dir/build.make
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_system.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_thread.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libpthread.so
icp4_after_feature_registration: /usr/lib/libpcl_common.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
icp4_after_feature_registration: /usr/lib/libpcl_kdtree.so
icp4_after_feature_registration: /usr/lib/libpcl_octree.so
icp4_after_feature_registration: /usr/lib/libpcl_search.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libqhull.so
icp4_after_feature_registration: /usr/lib/libpcl_surface.so
icp4_after_feature_registration: /usr/lib/libpcl_sample_consensus.so
icp4_after_feature_registration: /usr/lib/libOpenNI.so
icp4_after_feature_registration: /usr/lib/libOpenNI2.so
icp4_after_feature_registration: /usr/lib/libpcl_io.so
icp4_after_feature_registration: /usr/lib/libpcl_filters.so
icp4_after_feature_registration: /usr/lib/libpcl_features.so
icp4_after_feature_registration: /usr/lib/libpcl_keypoints.so
icp4_after_feature_registration: /usr/lib/libpcl_registration.so
icp4_after_feature_registration: /usr/lib/libpcl_segmentation.so
icp4_after_feature_registration: /usr/lib/libpcl_recognition.so
icp4_after_feature_registration: /usr/lib/libpcl_visualization.so
icp4_after_feature_registration: /usr/lib/libpcl_people.so
icp4_after_feature_registration: /usr/lib/libpcl_outofcore.so
icp4_after_feature_registration: /usr/lib/libpcl_tracking.so
icp4_after_feature_registration: /usr/lib/libpcl_apps.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_system.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_thread.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libpthread.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libqhull.so
icp4_after_feature_registration: /usr/lib/libOpenNI.so
icp4_after_feature_registration: /usr/lib/libOpenNI2.so
icp4_after_feature_registration: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
icp4_after_feature_registration: /usr/lib/libvtkGenericFiltering.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkGeovis.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkCharts.so.5.8.0
icp4_after_feature_registration: /usr/lib/libpcl_common.so
icp4_after_feature_registration: /usr/lib/libpcl_kdtree.so
icp4_after_feature_registration: /usr/lib/libpcl_octree.so
icp4_after_feature_registration: /usr/lib/libpcl_search.so
icp4_after_feature_registration: /usr/lib/libpcl_surface.so
icp4_after_feature_registration: /usr/lib/libpcl_sample_consensus.so
icp4_after_feature_registration: /usr/lib/libpcl_io.so
icp4_after_feature_registration: /usr/lib/libpcl_filters.so
icp4_after_feature_registration: /usr/lib/libpcl_features.so
icp4_after_feature_registration: /usr/lib/libpcl_keypoints.so
icp4_after_feature_registration: /usr/lib/libpcl_registration.so
icp4_after_feature_registration: /usr/lib/libpcl_segmentation.so
icp4_after_feature_registration: /usr/lib/libpcl_recognition.so
icp4_after_feature_registration: /usr/lib/libpcl_visualization.so
icp4_after_feature_registration: /usr/lib/libpcl_people.so
icp4_after_feature_registration: /usr/lib/libpcl_outofcore.so
icp4_after_feature_registration: /usr/lib/libpcl_tracking.so
icp4_after_feature_registration: /usr/lib/libpcl_apps.so
icp4_after_feature_registration: /usr/lib/libvtkViews.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkInfovis.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkWidgets.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkVolumeRendering.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkHybrid.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkParallel.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkRendering.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkImaging.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkGraphics.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkIO.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkFiltering.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtkCommon.so.5.8.0
icp4_after_feature_registration: /usr/lib/libvtksys.so.5.8.0
icp4_after_feature_registration: CMakeFiles/icp4_after_feature_registration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable icp4_after_feature_registration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/icp4_after_feature_registration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/icp4_after_feature_registration.dir/build: icp4_after_feature_registration
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/build

CMakeFiles/icp4_after_feature_registration.dir/requires: CMakeFiles/icp4_after_feature_registration.dir/src/icp4_after_feature_registration.cpp.o.requires
CMakeFiles/icp4_after_feature_registration.dir/requires: CMakeFiles/icp4_after_feature_registration.dir/src/visualize_correspondences.cpp.o.requires
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/requires

CMakeFiles/icp4_after_feature_registration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/icp4_after_feature_registration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/clean

CMakeFiles/icp4_after_feature_registration.dir/depend:
	cd /home/kazuhiro/PCL/tutorial/ICP-test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kazuhiro/PCL/tutorial/ICP-test /home/kazuhiro/PCL/tutorial/ICP-test /home/kazuhiro/PCL/tutorial/ICP-test/build /home/kazuhiro/PCL/tutorial/ICP-test/build /home/kazuhiro/PCL/tutorial/ICP-test/build/CMakeFiles/icp4_after_feature_registration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/icp4_after_feature_registration.dir/depend

