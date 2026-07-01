#include "ekf_slam/EKF_Pose.h"
#include <cmath>

EKFPose::EKFPose() {
    x_ = Eigen::Vector3d::Zero();
    P_ = Eigen::Matrix3d::Identity() * 1e-5;
    Q_ = Eigen::Matrix3d::Identity();
}

void EKFPose::setProcessNoise(double noise_v, double noise_vy, double noise_omega) {
    Q_(0,0) = noise_v * noise_v;
    Q_(1,1) = noise_vy * noise_vy;
    Q_(2,2) = noise_omega * noise_omega;
}

double EKFPose::normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void EKFPose::predict(double v, double vy, double omega, double dt) {
    if (std::isnan(v) || std::isnan(vy) || std::isnan(omega)) return;

    double theta = x_(2);
    double s = std::sin(theta);
    double c = std::cos(theta);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();

    // 1. Aggiornamento dello Stato (Posizione e Orientazione)
    // 2. Calcolo dello Jacobiano (F) per propagare l'incertezza
    if (std::abs(omega) > 1e-4) {
        double s_next = std::sin(theta + omega * dt);
        double c_next = std::cos(theta + omega * dt);

        // Traiettoria curvilinea esatta
        x_(0) += (v / omega) * (s_next - s) + (vy / omega) * (c_next - c);
        x_(1) += (v / omega) * (-c_next + c) + (vy / omega) * (s_next - s);
        x_(2) += omega * dt;

        F(0, 2) = (v / omega) * (c_next - c) - (vy / omega) * (s_next - s);
        F(1, 2) = (v / omega) * (s_next - s) + (vy / omega) * (c_next - c);
    } else {
        // Approssimazione per moto quasi rettilineo
        double theta_mid = theta + omega * dt * 0.5;
        double s_mid = std::sin(theta_mid);
        double c_mid = std::cos(theta_mid);

        x_(0) += (v * c_mid - vy * s_mid) * dt;
        x_(1) += (v * s_mid + vy * c_mid) * dt;
        x_(2) += omega * dt;

        F(0, 2) = (-v * s_mid - vy * c_mid) * dt;
        F(1, 2) = ( v * c_mid - vy * s_mid) * dt;
    }
    
    x_(2) = normalizeAngle(x_(2));

    // 3. Mappatura del rumore (Matrice W)
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    W(0,0) = c * dt; W(0,1) = -s * dt; 
    W(1,0) = s * dt; W(1,1) = c * dt;  
    W(2,2) = dt;      

    // 4. Aggiornamento della Covarianza P
    P_ = F * P_ * F.transpose() + W * Q_ * W.transpose();
}