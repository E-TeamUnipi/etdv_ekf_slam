#ifndef EKF_POSE_H
#define EKF_POSE_H

#include <Eigen/Dense>

class EKFPose {
public:
    EKFPose();
    void setProcessNoise(double noise_x, double noise_y, double noise_yaw, double noise_ax, double noise_ay, double noise_omega, double noise_lx, double noise_ly);
    void setState(const Eigen::VectorXd& state) { x_ = state; }
    void setCovariance(const Eigen::MatrixXd& cov) { P_ = cov; }
    void predict(double ax, double ay, double gyro_z, double dt);
    void correctPosition(int matched_id, double local_x_meas, double local_y_meas);
    void addNewLandmark(double local_x, double local_y);
    int  dataAssociation(double local_x, double local_y, double mahalanobis_th);
    
    Eigen::MatrixXd getCovariance() const { return P_; }
    Eigen::VectorXd getState() const { return x_; }
    Eigen::Matrix2d getR() const { return R_; }

private:

    static constexpr int VEHICLE_STATE_DIM = 6;  // [x, y, theta, vx, vy, omega]
    static constexpr int LANDMARK_DIM = 2;

    Eigen::VectorXd x_; 
    Eigen::MatrixXd P_; // 6x6 Covariance
    Eigen::MatrixXd Q_; // 6x6 Noise
    Eigen::MatrixXd R_; // measurement covariance
    double normalizeAngle(double a);
};
#endif