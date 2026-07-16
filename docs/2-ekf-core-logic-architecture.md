# EKF Core Logic & Architecture

The filter is designed to handle a dynamically growing state. We strictly avoid relying on wheel odometry (encoders) due to longitudinal slip, non-linear dynamic radius variations, and unobservable slip angles, which would inherently corrupt the measurement step.

## Dynamic State Vector and EKF implementation
The state vector expands dynamically whenever a new, unmapped cone is observed:
$$x = \left[ x, y, \theta, v_x, v_y, b_\omega, m_{1x}, m_{1y}, \dots, m_{nx}, m_{ny} \right]^T$$
Where $[x, y, \theta]$ is the vehicle pose, $[v_x, v_y]$ the velocities, $b_\omega$ the yaw rate bias, and $[m_{ix}, m_{iy}]$ the 2D coordinates of the $i$-th mapped cone.

Every IMU data collected triggers `imuCallback` and the prediction step:
$$\hat{x}_{k \mid k-1} = f(\hat{x}_{k-1 \mid k-1}, u_k)$$ $$P_{k \mid k-1} = F P_{k-1 \mid k-1} F^T + Q$$

with:
- $F$ the jacobian matrix of the kimatic model calculated at each step as the linearization at the current equilibrium point.
- $P$ the state covariance matrix. 
- $Q$ the process covariance matrix.

On the other hand every cones detected triggers `conesCallback` and the correction step:
$$S_k = H P_{k \mid k-1} H^T + R$$ $$K_k = P_{k \mid k-1} H^T S_k^{-1}$$ $$\hat{x}_{k \mid k} = \hat{x}_{k \mid k-1} + K_k(z_k - h(\hat{x}_{k \mid k-1}))$$ $$P_{k \mid k} = (I - K_k H) P_{k \mid k-1}$$

with:
- $H$ the jacobian matrix of the meauserment model calculated at each step as the linearization at the current equilibrium point.
- $R$ the measurement covariance matrix. 
- $S$ the innovation covariance matrix.

The kinematic model uses acceleration vector $u_k = \left[ a_x, a_y, \omega_{z} \right]^T$ as source of control and updates the state vector through these equation:
- global position update:
$$x_k = x_{k-1} + (v_{x,k-1} \cos\theta_{k-1} - v_{y,k-1} \sin\theta_{k-1}) \Delta t$$ $$y_k = y_{k-1} + (v_{x,k-1} \sin\theta_{k-1} + v_{y,k-1} \cos\theta_{k-1}) \Delta t$$ $$\theta_k = \theta_{k-1} + (\omega_{imu} - b_{\omega,k-1}) \Delta t$$
- body-frame dynamics
$$v_{x,k} = v_{x,k-1} + a_x \Delta t$$ $$v_{y,k} = v_{y,k-1} + a_y \Delta t$$
- bias evolution and landmark update (static)
$$b_{\omega,k} = b_{\omega,k-1}$$ $$m_{i,k} = m_{i,k-1} \quad \forall i \in \{1, \dots, N\}$$

The measurement model maps predicted state into measurement space such that is possibale to obtain the innovation between the expected cone position ad the one measured:
- global distance between cone and car:
$$\Delta x = m_{ix} - x_k$$ $$\Delta y = m_{iy} - y_k$$
- distance mapped in the measurment space:
$$\hat{z}_{x,i} = \Delta x \cos\theta_k + \Delta y \sin\theta_k$$ $$\hat{z}_{y,i} = -\Delta x \sin\theta_k + \Delta y \cos\theta_k$$

$\theta_{mid}$, used for the trapezoidal integration method, is used as equilibrium poit for the Jacobian matrixes calculation.

## Data Association
Mahalanobis threshold is used to solve data association problem. Each cone detected is compared through this threshold with each cone detected since that moment thanks to `addNewLandmark` function, taking into account innovation and its relative covariance:

$$\nu_k = z_k - h(\hat{x}_{k\vert{}k-1})$$
$$D_M^2 = \nu_k^T S_k^{-1} \nu_k$$

This makes possible to get correct association even during long run-time preventig filter to diverge.

[next ->](3-latency-reduction-and-optimization.md)

