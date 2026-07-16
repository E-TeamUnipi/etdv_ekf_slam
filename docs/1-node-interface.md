## Node Interface

The node communicates via standard ROS 2 topics and is fully configurable through a .yaml parameter file.

# Subscribed Topics

- `pacsim/imu/cog_imu` (`sensor_msgs/Imu`): High-frequency inertial data used for the prediction step (Dead-Reckoning).

- `/pacsim/perceptiop/livox_front/landmarks` (`pacsim/msg/PerceptionDetection`): array of the local positions of the cones detected.

# Published Topics

The core node, `ekf_node` is focused on calculation and, by using bridge node, `tf_to_pose` debug data is displayed real-time throug specific topic
Here's the `ekf_node` main tpics:
- `/ekf/ekf_pose_only` (`geometry_msgs/PoseStamped`): Estimated 2D pose (X, Y, $\theta$) of the vehicle.
- `/ekf/odometry` ([nav_msgs/msg/odometry]) Estimated pose used for tf transformation.
- `/ekf/map_cones` (visualization_msgs/MarkerArray): Global map of the estimated cone positions.

Here's the `tf_to_pose` main tpics:
- `/ekf/rmse` (`std_msgs/Float64`): Real-time Root Mean Square Error for pose estimation.
- `/ekf/map_rmse` (`std_msgs/Float64`): Real-time Root Mean Square Error for map estimation.
- `/ekf/imu_latency_ms` (`std_msgs/Float64`): Real-Time prediction latency.
- `/ekf/latency_ms` (`std_msgs/Float64`): Real-Time correction latency.

# Key Parameters (.yaml Configuration)

The filter tuning is completely isolated from the source code. 
Key parameters include:

Process noise covariance entries (Matrix Q):
- `process_noise_v`  
- `process_noise_vy` 
- `process_noise_omega` 

Measurement noise covariance (Matrix R):
- `lidar_noise_x` 
- `lidar_noise_y`

Thresholds:
- `association_threshold`: Mahalanobis distance limit for data association.
- `max_distance`: Euclidean cut-off radius (in meters) for spatial filtering.

[next ->](2-ekf-core-logic-architecture.md)