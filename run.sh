#!/bin/bash
source /opt/ros/humble/setup.bash

# # 1. Pulisce SOLO la cache dei pacchetti a cui stai lavorando (NON tutto l'install!)
# echo "Cleaning development cache..."
# rm -rf /ros2_ws/build/ekf_slam /ros2_ws/install/ekf_slam
# rm -rf /ros2_ws/build/circular_controller /ros2_ws/install/circular_controller

# 2. Compila tutto il necessario, ma IGNORA esplicitamente il pacchetto rotto
echo "Building workspace..."
cd /ros2_ws
colcon build --symlink-install --packages-ignore etdv_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Source dell'ambiente
source /ros2_ws/install/setup.bash

# 4. Lancio dei nodi
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
sleep 2

ros2 launch circular_controller circular_controller_feedback.launch.py &
sleep 2

ros2 launch ekf_slam ekf_slam.launch.py &
sleep 2

ros2 launch pacsim example.launch.py