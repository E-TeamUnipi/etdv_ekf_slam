#ifndef EKF_POSE_H
#define EKF_POSE_H

#include <Eigen/Dense>

class EKFPose {
public:
    EKFPose();
    void setProcessNoise(double noise_ax, double noise_ay, double noise_omega);
    void predict(double ax, double ay, double dt);
    void correctGyro(double z_gyro, double& out_y, double& out_k);
    
    Eigen::MatrixXd getCovariance() const { return P_; }
    Eigen::VectorXd getState() const { return x_; }

private:
    Eigen::VectorXd x_; // [x, y, theta, vx, vy, omega]
    Eigen::MatrixXd P_; // 6x6 Covariance
    Eigen::MatrixXd Q_; // 6x6 Noise
    Eigen::MatrixXd R_; // measurement covariance
    double normalizeAngle(double a);
};
#endif