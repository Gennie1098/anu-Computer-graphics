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
CMAKE_SOURCE_DIR = /home/anucg/Desktop/HW2/Code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anucg/Desktop/HW2/Code/build

# Include any dependencies generated for this target.
include CMakeFiles/task3.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/task3.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/task3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/task3.dir/flags.make

CMakeFiles/task3.dir/task3.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/task3.cpp.o: ../task3.cpp
CMakeFiles/task3.dir/task3.cpp.o: CMakeFiles/task3.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anucg/Desktop/HW2/Code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/task3.dir/task3.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/task3.dir/task3.cpp.o -MF CMakeFiles/task3.dir/task3.cpp.o.d -o CMakeFiles/task3.dir/task3.cpp.o -c /home/anucg/Desktop/HW2/Code/task3.cpp

CMakeFiles/task3.dir/task3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/task3.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anucg/Desktop/HW2/Code/task3.cpp > CMakeFiles/task3.dir/task3.cpp.i

CMakeFiles/task3.dir/task3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/task3.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anucg/Desktop/HW2/Code/task3.cpp -o CMakeFiles/task3.dir/task3.cpp.s

CMakeFiles/task3.dir/rasterizer.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/rasterizer.cpp.o: ../rasterizer.cpp
CMakeFiles/task3.dir/rasterizer.cpp.o: CMakeFiles/task3.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anucg/Desktop/HW2/Code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/task3.dir/rasterizer.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/task3.dir/rasterizer.cpp.o -MF CMakeFiles/task3.dir/rasterizer.cpp.o.d -o CMakeFiles/task3.dir/rasterizer.cpp.o -c /home/anucg/Desktop/HW2/Code/rasterizer.cpp

CMakeFiles/task3.dir/rasterizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/rasterizer.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anucg/Desktop/HW2/Code/rasterizer.cpp > CMakeFiles/task3.dir/rasterizer.cpp.i

CMakeFiles/task3.dir/rasterizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/rasterizer.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anucg/Desktop/HW2/Code/rasterizer.cpp -o CMakeFiles/task3.dir/rasterizer.cpp.s

CMakeFiles/task3.dir/Triangle.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/Triangle.cpp.o: ../Triangle.cpp
CMakeFiles/task3.dir/Triangle.cpp.o: CMakeFiles/task3.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anucg/Desktop/HW2/Code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/task3.dir/Triangle.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/task3.dir/Triangle.cpp.o -MF CMakeFiles/task3.dir/Triangle.cpp.o.d -o CMakeFiles/task3.dir/Triangle.cpp.o -c /home/anucg/Desktop/HW2/Code/Triangle.cpp

CMakeFiles/task3.dir/Triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/Triangle.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anucg/Desktop/HW2/Code/Triangle.cpp > CMakeFiles/task3.dir/Triangle.cpp.i

CMakeFiles/task3.dir/Triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/Triangle.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anucg/Desktop/HW2/Code/Triangle.cpp -o CMakeFiles/task3.dir/Triangle.cpp.s

CMakeFiles/task3.dir/Texture.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/Texture.cpp.o: ../Texture.cpp
CMakeFiles/task3.dir/Texture.cpp.o: CMakeFiles/task3.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anucg/Desktop/HW2/Code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/task3.dir/Texture.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/task3.dir/Texture.cpp.o -MF CMakeFiles/task3.dir/Texture.cpp.o.d -o CMakeFiles/task3.dir/Texture.cpp.o -c /home/anucg/Desktop/HW2/Code/Texture.cpp

CMakeFiles/task3.dir/Texture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/Texture.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anucg/Desktop/HW2/Code/Texture.cpp > CMakeFiles/task3.dir/Texture.cpp.i

CMakeFiles/task3.dir/Texture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/Texture.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anucg/Desktop/HW2/Code/Texture.cpp -o CMakeFiles/task3.dir/Texture.cpp.s

# Object files for target task3
task3_OBJECTS = \
"CMakeFiles/task3.dir/task3.cpp.o" \
"CMakeFiles/task3.dir/rasterizer.cpp.o" \
"CMakeFiles/task3.dir/Triangle.cpp.o" \
"CMakeFiles/task3.dir/Texture.cpp.o"

# External object files for target task3
task3_EXTERNAL_OBJECTS =

task3: CMakeFiles/task3.dir/task3.cpp.o
task3: CMakeFiles/task3.dir/rasterizer.cpp.o
task3: CMakeFiles/task3.dir/Triangle.cpp.o
task3: CMakeFiles/task3.dir/Texture.cpp.o
task3: CMakeFiles/task3.dir/build.make
task3: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_alphamat.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_barcode.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_mcc.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_rapid.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4d
task3: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4d
task3: CMakeFiles/task3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anucg/Desktop/HW2/Code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable task3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/task3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/task3.dir/build: task3
.PHONY : CMakeFiles/task3.dir/build

CMakeFiles/task3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/task3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/task3.dir/clean

CMakeFiles/task3.dir/depend:
	cd /home/anucg/Desktop/HW2/Code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anucg/Desktop/HW2/Code /home/anucg/Desktop/HW2/Code /home/anucg/Desktop/HW2/Code/build /home/anucg/Desktop/HW2/Code/build /home/anucg/Desktop/HW2/Code/build/CMakeFiles/task3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/task3.dir/depend
