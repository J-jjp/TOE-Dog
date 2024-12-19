cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(A1)

link_directories(/home/jiaojunpeng/mujoco-3.1.4/lib)

link_directories(/home/jiaojunpeng/mujoco-3.1.4/simulate)

# 设置 Mujoco 路径
# 查找 Eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Could not find Eigen3")
endif()

# 收集源文件
file(GLOB DOG_SRCS "src/FSM/*.cpp"
"src/interface/*.cpp"
"main.cpp")
# 如果还有其他源文件目录，可以在这里继续添加
# file(GLOB_RECURSE ADDITIONAL_SRCS "src/FCM/FCM.cpp")
# set(DOG_SRCS ${DOG_SRCS} ${ADDITIONAL_SRCS})

# 添加包含目录
# find_package(GLOB  ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc)
message(INFO ${DOG_SRCS})

include_directories(include  ${EIGEN3_INCLUDE_DIR}
/home/jiaojunpeng/mujoco-3.1.4/simulate /home/jiaojunpeng/mujoco-3.1.4/include
/home/jiaojunpeng/my_dog/dog_control/imgui
/home/jiaojunpeng/my_dog/dog_control/imgui/backends include/interface include/message include/FSM)


# 添加可执行文件
add_executable(dog ${DOG_SRCS})

# 链接库
target_link_libraries(dog  /home/jiaojunpeng/mujoco-3.1.4/lib/libmujoco.so glut GL GLU glfw)