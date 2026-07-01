#ifndef EKF_POSE_H
#define EKF_POSE_H

#include <Eigen/Dense>

class EKFPose {
public:
    EKFPose();

    // Imposta le varianze (deviazione standard al quadrato) dei sensori
    void setProcessNoise(double noise_v, double noise_vy, double noise_omega);

    // Integrazione cinematica esatta
    void predict(double v, double vy, double omega, double dt);

    // Getter
    Eigen::Vector3d getState() const { return x_; }
    Eigen::Matrix3d getCovariance() const { return P_; }

private:
    Eigen::Vector3d x_;  // Stato: [x, y, theta]
    Eigen::Matrix3d P_;  // Matrice di covarianza (3x3)
    Eigen::Matrix3d Q_;  // Matrice di rumore di processo

    double normalizeAngle(double a);
};

#endif // EKF_POSE_H