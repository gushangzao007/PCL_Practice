# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peter/Projects/PclTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/Projects/PclTest/build

# Include any dependencies generated for this target.
include CMakeFiles/FastTrangle.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/FastTrangle.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FastTrangle.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FastTrangle.dir/flags.make

CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o: CMakeFiles/FastTrangle.dir/flags.make
CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o: ../FastTrangle.cpp
CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o: CMakeFiles/FastTrangle.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peter/Projects/PclTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o -MF CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o.d -o CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o -c /home/peter/Projects/PclTest/FastTrangle.cpp

CMakeFiles/FastTrangle.dir/FastTrangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FastTrangle.dir/FastTrangle.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peter/Projects/PclTest/FastTrangle.cpp > CMakeFiles/FastTrangle.dir/FastTrangle.cpp.i

CMakeFiles/FastTrangle.dir/FastTrangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FastTrangle.dir/FastTrangle.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peter/Projects/PclTest/FastTrangle.cpp -o CMakeFiles/FastTrangle.dir/FastTrangle.cpp.s

# Object files for target FastTrangle
FastTrangle_OBJECTS = \
"CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o"

# External object files for target FastTrangle
FastTrangle_EXTERNAL_OBJECTS =

FastTrangle: CMakeFiles/FastTrangle.dir/FastTrangle.cpp.o
FastTrangle: CMakeFiles/FastTrangle.dir/build.make
FastTrangle: /usr/local/lib/libpcl_surface.so
FastTrangle: /usr/local/lib/libpcl_keypoints.so
FastTrangle: /usr/local/lib/libpcl_tracking.so
FastTrangle: /usr/local/lib/libpcl_recognition.so
FastTrangle: /usr/local/lib/libpcl_stereo.so
FastTrangle: /usr/local/lib/libpcl_outofcore.so
FastTrangle: /usr/local/lib/libpcl_people.so
FastTrangle: /usr/lib/libOpenNI.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
FastTrangle: /usr/lib/libNxLib64.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
FastTrangle: /usr/local/lib/libpcl_registration.so
FastTrangle: /usr/local/lib/libpcl_segmentation.so
FastTrangle: /usr/local/lib/libpcl_features.so
FastTrangle: /usr/local/lib/libpcl_filters.so
FastTrangle: /usr/local/lib/libpcl_sample_consensus.so
FastTrangle: /usr/local/lib/libpcl_ml.so
FastTrangle: /usr/local/lib/libpcl_visualization.so
FastTrangle: /usr/local/lib/libpcl_search.so
FastTrangle: /usr/local/lib/libpcl_kdtree.so
FastTrangle: /usr/local/lib/libpcl_io.so
FastTrangle: /usr/local/lib/libpcl_octree.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libpcap.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libpng.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libz.so
FastTrangle: /usr/lib/libOpenNI.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
FastTrangle: /usr/lib/libNxLib64.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkIOCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libfreetype.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkIOImage-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingUI-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkkissfft-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libGLEW.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libX11.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.15.3
FastTrangle: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
FastTrangle: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
FastTrangle: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-9.1.so.9.1.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libtbb.so.12.5
FastTrangle: /usr/lib/x86_64-linux-gnu/libvtksys-9.1.so.9.1.0
FastTrangle: /usr/local/lib/libpcl_common.so
FastTrangle: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.74.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
FastTrangle: /usr/lib/x86_64-linux-gnu/libqhull_r.so.8.0.2
FastTrangle: CMakeFiles/FastTrangle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peter/Projects/PclTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FastTrangle"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FastTrangle.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FastTrangle.dir/build: FastTrangle
.PHONY : CMakeFiles/FastTrangle.dir/build

CMakeFiles/FastTrangle.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FastTrangle.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FastTrangle.dir/clean

CMakeFiles/FastTrangle.dir/depend:
	cd /home/peter/Projects/PclTest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/Projects/PclTest /home/peter/Projects/PclTest /home/peter/Projects/PclTest/build /home/peter/Projects/PclTest/build /home/peter/Projects/PclTest/build/CMakeFiles/FastTrangle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FastTrangle.dir/depend

