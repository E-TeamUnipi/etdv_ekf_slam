cmake_minimum_required(VERSION 3.8)
project(circular_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pacsim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# Add executable
add_executable(circular_controller_node src/circular_controller_node.cpp)
ament_target_dependencies(circular_controller_node
  rclcpp
  pacsim
  std_msgs
  geometry_msgs
)

# Add feedback executable
add_executable(circular_controller_feedback_node src/circular_controller_feedback_node.cpp)
ament_target_dependencies(circular_controller_feedback_node
  rclcpp
  pacsim
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_msgs
)

# Install targets
install(TARGETS
  circular_controller_node
  circular_controller_feedback_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
