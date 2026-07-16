# Latency Reduction & Optimization
A naive implementation of EKF-SLAM suffers from an $O(N^2)$ computational bottleneck as the covariance matrix expands with every new cone. 
To ensure strict Real-Time performance, the following optimizations were implemented:
- **Dense Block Memory Extraction**: 
Sparse matrix libraries were replaced with direct memory access using `Eigen::Matrix::block`. This provides $O(1)$ time complexity for extracting and updating cross-covariance sub-matrices.
- **Dynamic Spatial Culling (15m Gate)**: 
A hard Euclidean boundary is applied to the perception data. Any measurement beyond 15 meters is ignored a priori, drastically cutting down the data association overhead and the filter divergence.
- **Rewind & Replay (Time Synchronization)**: 
To compensate for LiDAR acquisition delays, the EKF state is historically rewound to the exact timestamp of the perception event, corrected, and then re-propagated forward using the IMU history buffer. 
- **Result**: 
These algorithmic improvements reduced the worst-case correction latency by ~95.5% (from an initial **129** ms down **to 5.8** ms on specific tracks).

Thanks to latency reduction, overall error for pose and map estimation was reduced. Furthermore consideration on test results are specified [here](4-current-test-results.md)

[next ->](4-current-test-results.md)