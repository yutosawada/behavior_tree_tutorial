cmake_minimum_required(VERSION 3.8)
project(behavior_tree_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Create include directory
include_directories(include)

# Add executable for tutorial 1
add_executable(tutorial_01_first_tree
  src/tutorial_01_first_tree.cpp
  src/dummy_nodes.cpp
)

ament_target_dependencies(tutorial_01_first_tree
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# Add executable for tutorial 4
add_executable(tutorial_04_sequence
  src/tutorial_04_sequence.cpp
  src/dummy_nodes.cpp
)

ament_target_dependencies(tutorial_04_sequence
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# Install targets
install(TARGETS
  tutorial_01_first_tree
  tutorial_04_sequence
  DESTINATION lib/${PROJECT_NAME}
)

# Install XML files
install(DIRECTORY
  trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
