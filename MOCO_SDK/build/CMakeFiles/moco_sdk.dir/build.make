# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/corvin/QT/MOCO_SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/corvin/QT/MOCO_SDK/build

# Include any dependencies generated for this target.
include CMakeFiles/moco_sdk.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/moco_sdk.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/moco_sdk.dir/flags.make

CMakeFiles/moco_sdk.dir/src/main.cpp.o: CMakeFiles/moco_sdk.dir/flags.make
CMakeFiles/moco_sdk.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/corvin/QT/MOCO_SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/moco_sdk.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/moco_sdk.dir/src/main.cpp.o -c /home/corvin/QT/MOCO_SDK/src/main.cpp

CMakeFiles/moco_sdk.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/moco_sdk.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/corvin/QT/MOCO_SDK/src/main.cpp > CMakeFiles/moco_sdk.dir/src/main.cpp.i

CMakeFiles/moco_sdk.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/moco_sdk.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/corvin/QT/MOCO_SDK/src/main.cpp -o CMakeFiles/moco_sdk.dir/src/main.cpp.s

CMakeFiles/moco_sdk.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/moco_sdk.dir/src/main.cpp.o.requires

CMakeFiles/moco_sdk.dir/src/main.cpp.o.provides: CMakeFiles/moco_sdk.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/moco_sdk.dir/build.make CMakeFiles/moco_sdk.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/moco_sdk.dir/src/main.cpp.o.provides

CMakeFiles/moco_sdk.dir/src/main.cpp.o.provides.build: CMakeFiles/moco_sdk.dir/src/main.cpp.o


CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o: CMakeFiles/moco_sdk.dir/flags.make
CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o: ../src/moco_sdk.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/corvin/QT/MOCO_SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o -c /home/corvin/QT/MOCO_SDK/src/moco_sdk.cpp

CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/corvin/QT/MOCO_SDK/src/moco_sdk.cpp > CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.i

CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/corvin/QT/MOCO_SDK/src/moco_sdk.cpp -o CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.s

CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.requires:

.PHONY : CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.requires

CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.provides: CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.requires
	$(MAKE) -f CMakeFiles/moco_sdk.dir/build.make CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.provides.build
.PHONY : CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.provides

CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.provides.build: CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o


CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o: CMakeFiles/moco_sdk.dir/flags.make
CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o: ../src/moco_sdk_uart.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/corvin/QT/MOCO_SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o -c /home/corvin/QT/MOCO_SDK/src/moco_sdk_uart.cpp

CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/corvin/QT/MOCO_SDK/src/moco_sdk_uart.cpp > CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.i

CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/corvin/QT/MOCO_SDK/src/moco_sdk_uart.cpp -o CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.s

CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.requires:

.PHONY : CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.requires

CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.provides: CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.requires
	$(MAKE) -f CMakeFiles/moco_sdk.dir/build.make CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.provides.build
.PHONY : CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.provides

CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.provides.build: CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o


# Object files for target moco_sdk
moco_sdk_OBJECTS = \
"CMakeFiles/moco_sdk.dir/src/main.cpp.o" \
"CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o" \
"CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o"

# External object files for target moco_sdk
moco_sdk_EXTERNAL_OBJECTS =

moco_sdk: CMakeFiles/moco_sdk.dir/src/main.cpp.o
moco_sdk: CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o
moco_sdk: CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o
moco_sdk: CMakeFiles/moco_sdk.dir/build.make
moco_sdk: CMakeFiles/moco_sdk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/corvin/QT/MOCO_SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable moco_sdk"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/moco_sdk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/moco_sdk.dir/build: moco_sdk

.PHONY : CMakeFiles/moco_sdk.dir/build

CMakeFiles/moco_sdk.dir/requires: CMakeFiles/moco_sdk.dir/src/main.cpp.o.requires
CMakeFiles/moco_sdk.dir/requires: CMakeFiles/moco_sdk.dir/src/moco_sdk.cpp.o.requires
CMakeFiles/moco_sdk.dir/requires: CMakeFiles/moco_sdk.dir/src/moco_sdk_uart.cpp.o.requires

.PHONY : CMakeFiles/moco_sdk.dir/requires

CMakeFiles/moco_sdk.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/moco_sdk.dir/cmake_clean.cmake
.PHONY : CMakeFiles/moco_sdk.dir/clean

CMakeFiles/moco_sdk.dir/depend:
	cd /home/corvin/QT/MOCO_SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/corvin/QT/MOCO_SDK /home/corvin/QT/MOCO_SDK /home/corvin/QT/MOCO_SDK/build /home/corvin/QT/MOCO_SDK/build /home/corvin/QT/MOCO_SDK/build/CMakeFiles/moco_sdk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/moco_sdk.dir/depend
