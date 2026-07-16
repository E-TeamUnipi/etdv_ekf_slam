# etdv_ekf_slam - Season 2025/26

## Package Overview
The `etdv_ekf_slam` package is a core component of the autonomous navigation pipeline. Its primary objective is to implement a robust Simultaneous Localization and Mapping (SLAM) algorithm based on an Extended Kalman Filter (EKF). 

Developed within the `pacsim_ws` workspace, this node is designed to process vehicle dynamics and perception data to achieve two simultaneous goals during track navigation:
*   **Real-Time Pose Estimation:** Continuously calculate the vehicle's 2D kinematic state (position, orientation, and velocities).
*   **Dynamic Mapping:** Accurately detect, associate, and map the positions of track cones (landmarks) in a global reference frame.

This package serves as the foundational localization layer, ensuring that the downstream control algorithms receive the low-latency, high-accuracy state estimations strictly required for high-performance autonomous racing.

Check documentation [here](docs/index.md) 
