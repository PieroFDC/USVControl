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
CMAKE_SOURCE_DIR = /home/piero/Documents/USV/USVControl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/piero/Documents/USV/USVControl/build

# Include any dependencies generated for this target.
include CMakeFiles/USVControl.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/USVControl.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/USVControl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/USVControl.dir/flags.make

CMakeFiles/USVControl.dir/src/main.cpp.o: CMakeFiles/USVControl.dir/flags.make
CMakeFiles/USVControl.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/USVControl.dir/src/main.cpp.o: CMakeFiles/USVControl.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piero/Documents/USV/USVControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/USVControl.dir/src/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/USVControl.dir/src/main.cpp.o -MF CMakeFiles/USVControl.dir/src/main.cpp.o.d -o CMakeFiles/USVControl.dir/src/main.cpp.o -c /home/piero/Documents/USV/USVControl/src/main.cpp

CMakeFiles/USVControl.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/USVControl.dir/src/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/piero/Documents/USV/USVControl/src/main.cpp > CMakeFiles/USVControl.dir/src/main.cpp.i

CMakeFiles/USVControl.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/USVControl.dir/src/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/piero/Documents/USV/USVControl/src/main.cpp -o CMakeFiles/USVControl.dir/src/main.cpp.s

# Object files for target USVControl
USVControl_OBJECTS = \
"CMakeFiles/USVControl.dir/src/main.cpp.o"

# External object files for target USVControl
USVControl_EXTERNAL_OBJECTS =

USVControl: CMakeFiles/USVControl.dir/src/main.cpp.o
USVControl: CMakeFiles/USVControl.dir/build.make
USVControl: /home/piero/Documents/Libs/fuzzylite/fuzzylite/debug/bin/libfuzzylite-debug.so
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
USVControl: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libncnn.a
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
USVControl: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.74.0
USVControl: /usr/lib/gcc/x86_64-linux-gnu/11/libgomp.so
USVControl: /usr/lib/x86_64-linux-gnu/libpthread.a
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libglslang.a
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libSPIRV.a
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libMachineIndependent.a
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libOGLCompiler.a
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libOSDependent.a
USVControl: /home/piero/Documents/Libs/ncnn/build/install/lib/libGenericCodeGen.a
USVControl: CMakeFiles/USVControl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/piero/Documents/USV/USVControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable USVControl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/USVControl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/USVControl.dir/build: USVControl
.PHONY : CMakeFiles/USVControl.dir/build

CMakeFiles/USVControl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/USVControl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/USVControl.dir/clean

CMakeFiles/USVControl.dir/depend:
	cd /home/piero/Documents/USV/USVControl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/piero/Documents/USV/USVControl /home/piero/Documents/USV/USVControl /home/piero/Documents/USV/USVControl/build /home/piero/Documents/USV/USVControl/build /home/piero/Documents/USV/USVControl/build/CMakeFiles/USVControl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/USVControl.dir/depend

