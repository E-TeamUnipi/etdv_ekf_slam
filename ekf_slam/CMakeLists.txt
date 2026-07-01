cmake_minimum_required(VERSION 3.8)
project(ekf_slam)

# -----------------------------
# Find packages
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

# -----------------------------
# Include directories
# -----------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# -----------------------------
# Library
# -----------------------------
add_library(ekf_lib src/EKF_Pose.cpp)
ament_target_dependencies(ekf_lib
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
)
target_include_directories(ekf_lib PUBLIC include ${EIGEN3_INCLUDE_DIR})

# -----------------------------
# Node executable
# -----------------------------
add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node ekf_lib)
ament_target_dependencies(ekf_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

# -----------------------------
# Install targets and headers
# -----------------------------
install(TARGETS ekf_node ekf_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# -----------------------------
# Export package
# -----------------------------
ament_package()


