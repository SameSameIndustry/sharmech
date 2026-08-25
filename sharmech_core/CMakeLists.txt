cmake_minimum_required(VERSION 3.16)
project(sharmech_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/motion_generator_node.cpp
  src/joy_teleop_node.cpp
  src/hardware_bridge_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::MotionGeneratorNode"
  EXECUTABLE motion_generator_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::JoyTeleopNode"
  EXECUTABLE joy_teleop_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sharmech_core::HardwareBridgeNode"
  EXECUTABLE hardware_bridge_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # 純粋なロジック (ROS ノードを起動しない)
  ament_add_gtest(test_trapezoidal_trajectory test/test_trapezoidal_trajectory.cpp)
  target_include_directories(test_trapezoidal_trajectory PRIVATE include)

  ament_add_gtest(test_udp_protocol test/test_udp_protocol.cpp)
  target_include_directories(test_udp_protocol PRIVATE include)

  # motion_generator_node のウォッチドッグ・クランプ・調停ロジック
  # (rclcpp ノードとしてパブリッシュ/サブスクライブして検証する)
  ament_add_gtest(test_motion_generator_node test/test_motion_generator_node.cpp)
  target_link_libraries(test_motion_generator_node ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE)
