cmake_minimum_required(VERSION 3.16)
project(sharmech_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vr_interface_node.cpp
  src/state_manager_node.cpp
  src/trajectory_generator_node.cpp
  src/kinematics_node.cpp
  src/hardware_bridge_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::VrInterfaceNode"
  EXECUTABLE vr_interface_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::StateManagerNode"
  EXECUTABLE state_manager_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::TrajectoryGeneratorNode"
  EXECUTABLE trajectory_generator_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::KinematicsNode"
  EXECUTABLE kinematics_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::HardwareBridgeNode"
  EXECUTABLE hardware_bridge_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
