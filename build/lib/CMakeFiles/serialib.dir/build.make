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
include lib/CMakeFiles/serialib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lib/CMakeFiles/serialib.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/CMakeFiles/serialib.dir/progress.make

# Include the compile flags for this target's objects.
include lib/CMakeFiles/serialib.dir/flags.make

lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.o: lib/CMakeFiles/serialib.dir/flags.make
lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.o: ../lib/serialib/serialib.cpp
lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.o: lib/CMakeFiles/serialib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piero/Documents/USV/USVControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.o"
	cd /home/piero/Documents/USV/USVControl/build/lib && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.o -MF CMakeFiles/serialib.dir/serialib/serialib.cpp.o.d -o CMakeFiles/serialib.dir/serialib/serialib.cpp.o -c /home/piero/Documents/USV/USVControl/lib/serialib/serialib.cpp

lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialib.dir/serialib/serialib.cpp.i"
	cd /home/piero/Documents/USV/USVControl/build/lib && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/piero/Documents/USV/USVControl/lib/serialib/serialib.cpp > CMakeFiles/serialib.dir/serialib/serialib.cpp.i

lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialib.dir/serialib/serialib.cpp.s"
	cd /home/piero/Documents/USV/USVControl/build/lib && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/piero/Documents/USV/USVControl/lib/serialib/serialib.cpp -o CMakeFiles/serialib.dir/serialib/serialib.cpp.s

# Object files for target serialib
serialib_OBJECTS = \
"CMakeFiles/serialib.dir/serialib/serialib.cpp.o"

# External object files for target serialib
serialib_EXTERNAL_OBJECTS =

lib/libserialib.a: lib/CMakeFiles/serialib.dir/serialib/serialib.cpp.o
lib/libserialib.a: lib/CMakeFiles/serialib.dir/build.make
lib/libserialib.a: lib/CMakeFiles/serialib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/piero/Documents/USV/USVControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libserialib.a"
	cd /home/piero/Documents/USV/USVControl/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/serialib.dir/cmake_clean_target.cmake
	cd /home/piero/Documents/USV/USVControl/build/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serialib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/CMakeFiles/serialib.dir/build: lib/libserialib.a
.PHONY : lib/CMakeFiles/serialib.dir/build

lib/CMakeFiles/serialib.dir/clean:
	cd /home/piero/Documents/USV/USVControl/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/serialib.dir/cmake_clean.cmake
.PHONY : lib/CMakeFiles/serialib.dir/clean

lib/CMakeFiles/serialib.dir/depend:
	cd /home/piero/Documents/USV/USVControl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/piero/Documents/USV/USVControl /home/piero/Documents/USV/USVControl/lib /home/piero/Documents/USV/USVControl/build /home/piero/Documents/USV/USVControl/build/lib /home/piero/Documents/USV/USVControl/build/lib/CMakeFiles/serialib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/CMakeFiles/serialib.dir/depend

