cmake_minimum_required(VERSION 3.8)
project(sharmech_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(kinematics_node    src/kinematics_node.cpp)
ament_auto_add_executable(arm_controller_node src/arm_controller_node.cpp)
ament_auto_add_executable(gripper_node        src/gripper_node.cpp)
ament_auto_add_executable(task_manager_node   src/task_manager_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
