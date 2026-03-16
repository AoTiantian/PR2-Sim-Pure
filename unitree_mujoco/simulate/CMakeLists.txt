cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco_pr2)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
  src/unitree_sdk2_bridge
)

link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
  src/unitree_sdk2_bridge/unitree_sdk2_bridge.cc
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  fmt
)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
