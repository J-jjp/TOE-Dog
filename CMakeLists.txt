cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(A1)

link_directories(./mujoco-3.2.5/lib)

link_directories(./mujoco-3.2.5/simulate)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED )
if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Could not find Eigen3")
endif()

include_directories(${TORCH_INCLUDE_DIRS})

# 收集源文件
file(GLOB DOG_SRCS "src/FSM/*.cpp"
"src/control/*.cpp"
"src/interface/*.cpp"
"src/common/*.cpp")
# 如果还有其他源文件目录，可以在这里继续添加
# file(GLOB_RECURSE ADDITIONAL_SRCS "src/FCM/FCM.cpp")
# set(DOG_SRCS ${DOG_SRCS} ${ADDITIONAL_SRCS})

# 添加包含目录
# find_package(GLOB  ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc)
message(INFO ${DOG_SRCS})

include_directories(include  ${EIGEN3_INCLUDE_DIR}
./mujoco-3.2.5/simulate ./mujoco-3.2.5/include
./imgui
./imgui/backends include/interface include/message include/FSM include/communication)


# 添加可执行文件
add_executable(dogsim ${DOG_SRCS} main.cpp)
add_executable(dogreal ${DOG_SRCS} mainreal.cpp)
# 链接库
target_link_libraries(dogsim  /home/toe/TOE-Dog/mujoco-3.2.5/lib/libmujoco.so glut GL GLU glfw pthread)
target_link_libraries(dogreal  /home/toe/TOE-Dog/mujoco-3.2.5/lib/libmujoco.so glut GL GLU glfw pthread)