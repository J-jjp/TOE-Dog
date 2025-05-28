cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(dogreal)
# 获取当前目录
get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF,OFF=simulate
#select auto
set(AUTO OFF)          

#select robot
set(ROBOT_TYPE TOE_2)  # 参数 TOE_1 TOE_2 TOE_3
#select robot Contest_type
set(CONTEST_TYPE BARRIER)  # 参数 SPEED（竞速） BARRIER（障碍） FIELD（越野）

#参数校验
if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "Must define PLATFORM_TYPE")
endif()
if(NOT DEFINED CONTEST_TYPE)
    message(FATAL_ERROR "Must define CONTEST_TYPE")
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(MNN_INCLUDE_DIRS "${CURRENT_DIR}/MNN/include")
set(MNN_LIBRARIES "${CURRENT_DIR}/MNN/libMNN.so")
find_package(Eigen3 REQUIRED )
# 收集源文件
file(GLOB DOG_SRCS 
    "src/FSM/*.cpp"
    "src/control/*.cpp"
    "src/common/*.cpp"
    "src/interface/KeyBoard.cpp" 
    "src/interface/WirelessHandle.cpp"
)
add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "TOE_1")
        find_package(catkin REQUIRED COMPONENTS
          roscpp
          sensor_msgs
          unitree_a1
        )
    elseif(${ROBOT_TYPE} STREQUAL "TOE_2")
        find_package(catkin REQUIRED COMPONENTS
          roscpp
          sensor_msgs
          damiao_msgs
        )
    elseif(${ROBOT_TYPE} STREQUAL "TOE_3")


    endif()
else()
    list(APPEND DOG_SRCS "src/interface/IOMujoco.cpp")
    find_package(glfw3 REQUIRED)
    link_directories(./mujoco-3.2.5/lib ./mujoco-3.2.5/simulate)
endif()


#determine robot type
if(${ROBOT_TYPE} STREQUAL "TOE_1")
    add_definitions(-DROBOT_TYPE_T1)
elseif(${ROBOT_TYPE} STREQUAL "TOE_2")
    add_definitions(-DROBOT_TYPE_T2)
elseif(${ROBOT_TYPE} STREQUAL "TOE_3")
    add_definitions(-DROBOT_TYPE_T3)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()


#determine robot type
if(${CONTEST_TYPE} STREQUAL "SPEED")
    add_definitions(-DCONTEST_TYPE_SPEED)
elseif(${CONTEST_TYPE} STREQUAL "BARRIER")
    add_definitions(-DCONTEST_TYPE_BARRIER)
elseif(${CONTEST_TYPE} STREQUAL "FIELD")
    add_definitions(-DCONTEST_TYPE_FIELD)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

#select auto
if(AUTO)
    add_definitions(-DCAUTO_REAL_ROBOT)
else()
endif()
# 打印当前目录
# message(STATUS "Current directory is: ${CURRENT_DIR}")




# 添加包含目录
message(INFO ${DOG_SRCS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include  
./mujoco-3.2.5/simulate ./mujoco-3.2.5/include
./imgui ./imgui/backends include/interface include/message 
 include/FSM include/communication ${MNN_INCLUDE_DIRS})


# 添加可执行文件
if(REAL_ROBOT)
  if(${ROBOT_TYPE} STREQUAL "TOE_1")
    add_executable(dogreal ${DOG_SRCS} mainreal.cpp)
    target_link_libraries(dogreal  ${CURRENT_DIR}/mujoco-3.2.5/lib/libmujoco.so glut GL GLU glfw pthread ${MNN_LIBRARIES} ${catkin_LIBRARIES})
  endif()
else()
  add_executable(dogsim ${DOG_SRCS} main.cpp)
  target_link_libraries(dogsim  ${CURRENT_DIR}/mujoco-3.2.5/lib/libmujoco.so glut GL GLU glfw pthread ${MNN_LIBRARIES} ${catkin_LIBRARIES})
endif()
