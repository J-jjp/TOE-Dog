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
CMAKE_SOURCE_DIR = /home/toe/TOE-Dog

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/toe/TOE-Dog/build

# Include any dependencies generated for this target.
include CMakeFiles/dogreal_dm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dogreal_dm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dogreal_dm.dir/flags.make

CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.o: ../src/FSM/FSM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.o -c /home/toe/TOE-Dog/src/FSM/FSM.cpp

CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/FSM/FSM.cpp > CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.i

CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/FSM/FSM.cpp -o CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.s

CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.o: ../src/FSM/FSMState.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.o -c /home/toe/TOE-Dog/src/FSM/FSMState.cpp

CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/FSM/FSMState.cpp > CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.i

CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/FSM/FSMState.cpp -o CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.s

CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.o: ../src/FSM/State_FixedStand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.o -c /home/toe/TOE-Dog/src/FSM/State_FixedStand.cpp

CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/FSM/State_FixedStand.cpp > CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.i

CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/FSM/State_FixedStand.cpp -o CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.s

CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.o: ../src/FSM/State_FreeStand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.o -c /home/toe/TOE-Dog/src/FSM/State_FreeStand.cpp

CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/FSM/State_FreeStand.cpp > CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.i

CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/FSM/State_FreeStand.cpp -o CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.s

CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.o: ../src/FSM/State_Passive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.o -c /home/toe/TOE-Dog/src/FSM/State_Passive.cpp

CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/FSM/State_Passive.cpp > CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.i

CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/FSM/State_Passive.cpp -o CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.s

CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.o: ../src/FSM/State_Rl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.o -c /home/toe/TOE-Dog/src/FSM/State_Rl.cpp

CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/FSM/State_Rl.cpp > CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.i

CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/FSM/State_Rl.cpp -o CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.s

CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.o: ../src/common/unitreeLeg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.o -c /home/toe/TOE-Dog/src/common/unitreeLeg.cpp

CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/common/unitreeLeg.cpp > CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.i

CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/common/unitreeLeg.cpp -o CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.s

CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.o: ../src/common/unitreeRobot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.o -c /home/toe/TOE-Dog/src/common/unitreeRobot.cpp

CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/common/unitreeRobot.cpp > CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.i

CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/common/unitreeRobot.cpp -o CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.s

CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.o: ../src/control/ControlFrame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.o -c /home/toe/TOE-Dog/src/control/ControlFrame.cpp

CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/control/ControlFrame.cpp > CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.i

CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/control/ControlFrame.cpp -o CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.s

CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.o: ../src/control/leg_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.o -c /home/toe/TOE-Dog/src/control/leg_control.cpp

CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/control/leg_control.cpp > CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.i

CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/control/leg_control.cpp -o CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.s

CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.o: ../src/control/rl_Inference.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.o -c /home/toe/TOE-Dog/src/control/rl_Inference.cpp

CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/control/rl_Inference.cpp > CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.i

CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/control/rl_Inference.cpp -o CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.s

CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.o: ../src/interface/IOMujoco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.o -c /home/toe/TOE-Dog/src/interface/IOMujoco.cpp

CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/interface/IOMujoco.cpp > CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.i

CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/interface/IOMujoco.cpp -o CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.s

CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.o: ../src/interface/IOROS\ .cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.o -c "/home/toe/TOE-Dog/src/interface/IOROS .cpp"

CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/toe/TOE-Dog/src/interface/IOROS .cpp" > CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.i

CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/toe/TOE-Dog/src/interface/IOROS .cpp" -o CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.s

CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.o: ../src/interface/IOROS_dm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.o -c /home/toe/TOE-Dog/src/interface/IOROS_dm.cpp

CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/interface/IOROS_dm.cpp > CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.i

CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/interface/IOROS_dm.cpp -o CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.s

CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.o: ../src/interface/IOReal_old.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.o -c /home/toe/TOE-Dog/src/interface/IOReal_old.cpp

CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/interface/IOReal_old.cpp > CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.i

CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/interface/IOReal_old.cpp -o CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.s

CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.o: ../src/interface/KeyBoard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.o -c /home/toe/TOE-Dog/src/interface/KeyBoard.cpp

CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/interface/KeyBoard.cpp > CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.i

CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/interface/KeyBoard.cpp -o CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.s

CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.o: ../src/interface/WirelessHandle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.o -c /home/toe/TOE-Dog/src/interface/WirelessHandle.cpp

CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/src/interface/WirelessHandle.cpp > CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.i

CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/src/interface/WirelessHandle.cpp -o CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.s

CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.o: CMakeFiles/dogreal_dm.dir/flags.make
CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.o: ../mainreal_dm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.o -c /home/toe/TOE-Dog/mainreal_dm.cpp

CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/toe/TOE-Dog/mainreal_dm.cpp > CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.i

CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/toe/TOE-Dog/mainreal_dm.cpp -o CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.s

# Object files for target dogreal_dm
dogreal_dm_OBJECTS = \
"CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.o" \
"CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.o" \
"CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.o"

# External object files for target dogreal_dm
dogreal_dm_EXTERNAL_OBJECTS =

dogreal_dm: CMakeFiles/dogreal_dm.dir/src/FSM/FSM.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/FSM/FSMState.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/FSM/State_FixedStand.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/FSM/State_FreeStand.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/FSM/State_Passive.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/FSM/State_Rl.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/common/unitreeLeg.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/common/unitreeRobot.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/control/ControlFrame.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/control/leg_control.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/control/rl_Inference.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/interface/IOMujoco.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/interface/IOROS_.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/interface/IOROS_dm.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/interface/IOReal_old.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/interface/KeyBoard.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/src/interface/WirelessHandle.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/mainreal_dm.cpp.o
dogreal_dm: CMakeFiles/dogreal_dm.dir/build.make
dogreal_dm: ../mujoco-3.2.5/lib/libmujoco.so
dogreal_dm: /usr/local/lib/libglfw3.a
dogreal_dm: ../MNN/libMNN.so
dogreal_dm: /opt/ros/noetic/lib/libroscpp.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/libpthread.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
dogreal_dm: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
dogreal_dm: /opt/ros/noetic/lib/librosconsole.so
dogreal_dm: /opt/ros/noetic/lib/librosconsole_log4cxx.so
dogreal_dm: /opt/ros/noetic/lib/librosconsole_backend_interface.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
dogreal_dm: /opt/ros/noetic/lib/libxmlrpcpp.so
dogreal_dm: /opt/ros/noetic/lib/libroscpp_serialization.so
dogreal_dm: /opt/ros/noetic/lib/librostime.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
dogreal_dm: /opt/ros/noetic/lib/libcpp_common.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
dogreal_dm: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
dogreal_dm: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
dogreal_dm: /usr/lib/x86_64-linux-gnu/librt.so
dogreal_dm: /usr/lib/x86_64-linux-gnu/libm.so
dogreal_dm: CMakeFiles/dogreal_dm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/toe/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Linking CXX executable dogreal_dm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dogreal_dm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dogreal_dm.dir/build: dogreal_dm

.PHONY : CMakeFiles/dogreal_dm.dir/build

CMakeFiles/dogreal_dm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dogreal_dm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dogreal_dm.dir/clean

CMakeFiles/dogreal_dm.dir/depend:
	cd /home/toe/TOE-Dog/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/toe/TOE-Dog /home/toe/TOE-Dog /home/toe/TOE-Dog/build /home/toe/TOE-Dog/build /home/toe/TOE-Dog/build/CMakeFiles/dogreal_dm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dogreal_dm.dir/depend

