# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/luigi/Documents/ARGoS/argos-AGGREGATION/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luigi/Documents/ARGoS/argos-AGGREGATION/build

# Utility rule file for kilobot_phototaxis_autogen.

# Include the progress variables for this target.
include examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/progress.make

examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/luigi/Documents/ARGoS/argos-AGGREGATION/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target kilobot_phototaxis"
	cd /home/luigi/Documents/ARGoS/argos-AGGREGATION/build/examples/controllers/kilobot_phototaxis && /usr/bin/cmake -E cmake_autogen /home/luigi/Documents/ARGoS/argos-AGGREGATION/build/examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir Release

kilobot_phototaxis_autogen: examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen
kilobot_phototaxis_autogen: examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/build.make

.PHONY : kilobot_phototaxis_autogen

# Rule to build all files generated by this target.
examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/build: kilobot_phototaxis_autogen

.PHONY : examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/build

examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/clean:
	cd /home/luigi/Documents/ARGoS/argos-AGGREGATION/build/examples/controllers/kilobot_phototaxis && $(CMAKE_COMMAND) -P CMakeFiles/kilobot_phototaxis_autogen.dir/cmake_clean.cmake
.PHONY : examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/clean

examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/depend:
	cd /home/luigi/Documents/ARGoS/argos-AGGREGATION/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luigi/Documents/ARGoS/argos-AGGREGATION/src /home/luigi/Documents/ARGoS/argos-AGGREGATION/src/examples/controllers/kilobot_phototaxis /home/luigi/Documents/ARGoS/argos-AGGREGATION/build /home/luigi/Documents/ARGoS/argos-AGGREGATION/build/examples/controllers/kilobot_phototaxis /home/luigi/Documents/ARGoS/argos-AGGREGATION/build/examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/controllers/kilobot_phototaxis/CMakeFiles/kilobot_phototaxis_autogen.dir/depend

