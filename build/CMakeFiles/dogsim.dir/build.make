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
CMAKE_SOURCE_DIR = /home/jiaojunpeng/my_dog/TOE-Dog

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jiaojunpeng/my_dog/TOE-Dog/build

# Include any dependencies generated for this target.
include CMakeFiles/dogsim.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dogsim.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dogsim.dir/flags.make

CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.o: ../src/FSM/FSM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/FSM.cpp

CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/FSM.cpp > CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.i

CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/FSM.cpp -o CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.s

CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.o: ../src/FSM/FSMState.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/FSMState.cpp

CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/FSMState.cpp > CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.i

CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/FSMState.cpp -o CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.s

CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.o: ../src/FSM/State_FixedStand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_FixedStand.cpp

CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_FixedStand.cpp > CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.i

CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_FixedStand.cpp -o CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.s

CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.o: ../src/FSM/State_FreeStand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_FreeStand.cpp

CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_FreeStand.cpp > CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.i

CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_FreeStand.cpp -o CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.s

CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.o: ../src/FSM/State_Passive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_Passive.cpp

CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_Passive.cpp > CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.i

CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_Passive.cpp -o CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.s

CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.o: ../src/FSM/State_Rl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_Rl.cpp

CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_Rl.cpp > CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.i

CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/FSM/State_Rl.cpp -o CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.s

CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.o: ../src/common/unitreeLeg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/common/unitreeLeg.cpp

CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/common/unitreeLeg.cpp > CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.i

CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/common/unitreeLeg.cpp -o CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.s

CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.o: ../src/common/unitreeRobot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/common/unitreeRobot.cpp

CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/common/unitreeRobot.cpp > CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.i

CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/common/unitreeRobot.cpp -o CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.s

CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.o: ../src/control/ControlFrame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/control/ControlFrame.cpp

CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/control/ControlFrame.cpp > CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.i

CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/control/ControlFrame.cpp -o CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.s

CMakeFiles/dogsim.dir/src/control/leg_control.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/control/leg_control.cpp.o: ../src/control/leg_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/dogsim.dir/src/control/leg_control.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/control/leg_control.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/control/leg_control.cpp

CMakeFiles/dogsim.dir/src/control/leg_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/control/leg_control.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/control/leg_control.cpp > CMakeFiles/dogsim.dir/src/control/leg_control.cpp.i

CMakeFiles/dogsim.dir/src/control/leg_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/control/leg_control.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/control/leg_control.cpp -o CMakeFiles/dogsim.dir/src/control/leg_control.cpp.s

CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.o: ../src/control/rl_Inference.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/control/rl_Inference.cpp

CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/control/rl_Inference.cpp > CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.i

CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/control/rl_Inference.cpp -o CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.s

CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.o: ../src/interface/IOMujoco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/IOMujoco.cpp

CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/IOMujoco.cpp > CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.i

CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/IOMujoco.cpp -o CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.s

CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.o: ../src/interface/IOReal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/IOReal.cpp

CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/IOReal.cpp > CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.i

CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/IOReal.cpp -o CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.s

CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.o: ../src/interface/KeyBoard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/KeyBoard.cpp

CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/KeyBoard.cpp > CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.i

CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/KeyBoard.cpp -o CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.s

CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.o: ../src/interface/WirelessHandle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/WirelessHandle.cpp

CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/WirelessHandle.cpp > CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.i

CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/src/interface/WirelessHandle.cpp -o CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.s

CMakeFiles/dogsim.dir/main.cpp.o: CMakeFiles/dogsim.dir/flags.make
CMakeFiles/dogsim.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/dogsim.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dogsim.dir/main.cpp.o -c /home/jiaojunpeng/my_dog/TOE-Dog/main.cpp

CMakeFiles/dogsim.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dogsim.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiaojunpeng/my_dog/TOE-Dog/main.cpp > CMakeFiles/dogsim.dir/main.cpp.i

CMakeFiles/dogsim.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dogsim.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiaojunpeng/my_dog/TOE-Dog/main.cpp -o CMakeFiles/dogsim.dir/main.cpp.s

# Object files for target dogsim
dogsim_OBJECTS = \
"CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.o" \
"CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.o" \
"CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.o" \
"CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.o" \
"CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.o" \
"CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.o" \
"CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.o" \
"CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.o" \
"CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.o" \
"CMakeFiles/dogsim.dir/src/control/leg_control.cpp.o" \
"CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.o" \
"CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.o" \
"CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.o" \
"CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.o" \
"CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.o" \
"CMakeFiles/dogsim.dir/main.cpp.o"

# External object files for target dogsim
dogsim_EXTERNAL_OBJECTS =

dogsim: CMakeFiles/dogsim.dir/src/FSM/FSM.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/FSM/FSMState.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/FSM/State_FixedStand.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/FSM/State_FreeStand.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/FSM/State_Passive.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/FSM/State_Rl.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/common/unitreeLeg.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/common/unitreeRobot.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/control/ControlFrame.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/control/leg_control.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/control/rl_Inference.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/interface/IOMujoco.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/interface/IOReal.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/interface/KeyBoard.cpp.o
dogsim: CMakeFiles/dogsim.dir/src/interface/WirelessHandle.cpp.o
dogsim: CMakeFiles/dogsim.dir/main.cpp.o
dogsim: CMakeFiles/dogsim.dir/build.make
dogsim: ../mujoco-3.2.5/lib/libmujoco.so
dogsim: /usr/local/lib/libglfw3.a
dogsim: ../MNN/libMNN.so
dogsim: /opt/ros/noetic/lib/libroscpp.so
dogsim: /usr/lib/x86_64-linux-gnu/libpthread.so
dogsim: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
dogsim: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
dogsim: /opt/ros/noetic/lib/librosconsole.so
dogsim: /opt/ros/noetic/lib/librosconsole_log4cxx.so
dogsim: /opt/ros/noetic/lib/librosconsole_backend_interface.so
dogsim: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
dogsim: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
dogsim: /opt/ros/noetic/lib/libxmlrpcpp.so
dogsim: /opt/ros/noetic/lib/libroscpp_serialization.so
dogsim: /opt/ros/noetic/lib/librostime.so
dogsim: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
dogsim: /opt/ros/noetic/lib/libcpp_common.so
dogsim: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
dogsim: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
dogsim: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
dogsim: /usr/lib/x86_64-linux-gnu/librt.so
dogsim: /usr/lib/x86_64-linux-gnu/libm.so
dogsim: CMakeFiles/dogsim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Linking CXX executable dogsim"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dogsim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dogsim.dir/build: dogsim

.PHONY : CMakeFiles/dogsim.dir/build

CMakeFiles/dogsim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dogsim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dogsim.dir/clean

CMakeFiles/dogsim.dir/depend:
	cd /home/jiaojunpeng/my_dog/TOE-Dog/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jiaojunpeng/my_dog/TOE-Dog /home/jiaojunpeng/my_dog/TOE-Dog /home/jiaojunpeng/my_dog/TOE-Dog/build /home/jiaojunpeng/my_dog/TOE-Dog/build /home/jiaojunpeng/my_dog/TOE-Dog/build/CMakeFiles/dogsim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dogsim.dir/depend
