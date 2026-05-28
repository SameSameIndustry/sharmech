cmake_minimum_required(VERSION 3.16)
project(sharmech_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kinematics_node.cpp
  src/arm_controller_node.cpp
  src/gripper_node.cpp
  src/task_manager_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::KinematicsNode"
  EXECUTABLE kinematics_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::ArmControllerNode"
  EXECUTABLE arm_controller_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::GripperNode"
  EXECUTABLE gripper_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::TaskManagerNode"
  EXECUTABLE task_manager_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
