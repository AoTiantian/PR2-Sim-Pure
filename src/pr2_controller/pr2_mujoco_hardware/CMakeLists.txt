cmake_minimum_required(VERSION 3.8)
project(pr2_mujoco_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(unitree_sdk2 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  pr2_mujoco_hardware
  SHARED
  src/pr2_mujoco_hardware.cpp
)

target_compile_features(pr2_mujoco_hardware PUBLIC cxx_std_17)

target_include_directories(pr2_mujoco_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pr2_mujoco_hardware>
)

ament_target_dependencies(
  pr2_mujoco_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(pr2_mujoco_hardware
  PUBLIC
    unitree_sdk2
)

target_compile_definitions(pr2_mujoco_hardware PRIVATE "PR2_MUJOCO_HARDWARE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface pr2_mujoco_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/pr2_mujoco_hardware
)

install(TARGETS pr2_mujoco_hardware
  EXPORT export_pr2_mujoco_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_pr2_mujoco_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
