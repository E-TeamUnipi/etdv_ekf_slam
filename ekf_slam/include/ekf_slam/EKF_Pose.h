#ifndef EKF_POSE_H
#define EKF_POSE_H

#include <Eigen/Dense>
#include <vector>

struct EKFDiagnostic {
    double timestamp;
    double residual_ax;
    double residual_ay;
};

class EKFPose {
public:
    EKFPose();
    void setProcessNoise(double noise_ax, double noise_ay, double noise_omega);
    // Predict ora accetta ax, ay (IMU)
    void predict(double ax, double ay, double dt);
    void correctGyro(double z_gyro, double& out_y, double& out_k);

    Eigen::VectorXd getState() const { return x_; }
    void logDiagnostic(double ts, double res_ax, double res_ay);
    std::vector<EKFDiagnostic> diagnostics;

private:
    Eigen::VectorXd x_; // [x, y, theta, vx, vy, omega]
    Eigen::MatrixXd P_; // 6x6 Covariance
    Eigen::MatrixXd Q_; // 6x6 Noise
    Eigen::MatrixXd R_; // measurment covariance
    double normalizeAngle(double a);
};
#endif