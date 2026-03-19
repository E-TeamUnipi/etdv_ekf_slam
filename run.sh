#!/bin/bash
source /opt/ros/humble/setup.bash

# Build workspace completo se non esiste
if [ ! -d "/ros2_ws/install" ]; then
    echo "Building ROS2 workspace..."
    cd /ros2_ws
    colcon build
fi

# Ricompila SEMPRE ekf_slam per includere le modifiche più recenti
echo "Building ekf_slam package..."
cd /ros2_ws
colcon build --packages-select ekf_slam
source /ros2_ws/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
sleep 2

ros2 launch circular_controller circular_controller_feedback.launch.py &
sleep 2

ros2 run ekf_slam ekf_node &
sleep 2

ros2 launch pacsim example.launch.py
