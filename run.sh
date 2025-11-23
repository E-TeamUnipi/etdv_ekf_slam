#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
sleep 2

ros2 launch circular_controller circular_controller_feedback.launch.py &
sleep 2

ros2 launch pacsim example.launch.py
