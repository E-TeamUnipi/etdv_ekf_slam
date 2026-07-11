#include "ekf_slam/EKF_Pose.h"
#include <cmath>

EKFPose::EKFPose() {
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    R_ = Eigen::MatrixXd::Identity(1, 1) * 0.01;
}   

void EKFPose::setProcessNoise(double nv, double nvy, double nw) {
    Q_.diagonal() << nv*nv, nv*nv, nw*nw, nvy*nvy, nvy*nvy, nw*nw;
}

double EKFPose::normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void EKFPose::predict(double ax, double ay, double gyro_z, double dt) {
    double th = x_(2);
    double vx = x_(3);
    double vy = x_(4);
    double b_w = x_(5); // Ora x(5) è il bias del giroscopio

    // La rotazione reale è quella misurata meno il bias stimato
    double omega = gyro_z - b_w;

    Eigen::VectorXd next_x(6);
    // Integrazione cinematica
    next_x(0) = x_(0) + (vx * std::cos(th) - vy * std::sin(th)) * dt;
    next_x(1) = x_(1) + (vx * std::sin(th) + vy * std::cos(th)) * dt;
    next_x(2) = normalizeAngle(x_(2) + omega * dt);
    next_x(3) = vx + (ax + omega * vy) * dt; 
    next_x(4) = vy + (ay - omega * vx) * dt; 
    next_x(5) = b_w; // Il bias evolve (o resta costante)

    // Aggiornamento dello Jacobiano F (6x6)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    
    // Derivate per X e Y rispetto a Theta
    F(0, 2) = (-vx * std::sin(th) - vy * std::cos(th)) * dt;
    F(1, 2) = ( vx * std::cos(th) - vy * std::sin(th)) * dt;
    
    // Derivate per Theta rispetto al bias (negativo perché omega = gyro - bias)
    F(2, 5) = -dt; 
    
    // Derivate velocità
    F(3, 4) = omega * dt;  F(3, 5) = -vy * dt; // Influenzato dal bias
    F(4, 3) = -omega * dt; F(4, 5) = vx * dt;  // Influenzato dal bias

    // F(0,3), F(0,4), F(1,3), F(1,4) rimangono come nel tuo codice originale
    F(0, 3) = std::cos(th) * dt; F(0, 4) = -std::sin(th) * dt;
    F(1, 3) = std::sin(th) * dt; F(1, 4) = std::cos(th) * dt;

    P_ = F * P_ * F.transpose() + Q_;
    x_ = next_x;
}

void EKFPose::correctGyro(double z_gyro, double& out_y, double& out_k) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
    H(0, 5) = 1.0; 
    Eigen::MatrixXd z(1, 1);
    z(0, 0) = z_gyro;

    Eigen::MatrixXd y_mat = z - H * x_; 
    double y = y_mat(0, 0); 
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    out_y = y;
    out_k = K(5, 0);

    x_ = x_ + K * y_mat;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P_ = (I - K * H) * P_;
}