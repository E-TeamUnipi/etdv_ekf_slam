## Installation Prerequisites

To build and run this node within the `pacsim_ws` workspace, ensure the following dependencies are installed:

*   **ROS 2:** (Humble/Iron)
*   **Eigen3:** Required for high-performance dense matrix operations and algebraic calculations.
*   **C++17 or higher:** Required for advanced memory management and standard library features.
*   **Colcon:** ROS 2 default build system.

Ensure the workspace is properly sourced before building:
```bash
colcon build --packages-select etdv_ekf_slam --cmake-ars -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
[next ->](1-node-interface.md)