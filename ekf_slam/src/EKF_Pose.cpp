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
    double b_w = x_(5);

    // La rotazione reale è quella misurata meno il bias stimato
    double omega = gyro_z - b_w;

    // --- INTEGRAZIONE AL 2° ORDINE (PUNTO MEDIO) ---
    // Invece di usare l'angolo iniziale, calcoliamo l'angolo a metà del dt
    double th_mid = th + omega * (dt / 2.0);

    Eigen::VectorXd next_x(6);
    // Usiamo th_mid per proiettare le velocità, simulando una traiettoria curva
    next_x(0) = x_(0) + (vx * std::cos(th_mid) - vy * std::sin(th_mid)) * dt;
    next_x(1) = x_(1) + (vx * std::sin(th_mid) + vy * std::cos(th_mid)) * dt;
    next_x(2) = normalizeAngle(th + omega * dt); // L'angolo finale avanza normalmente
    next_x(3) = vx + (ax + omega * vy) * dt; 
    next_x(4) = vy + (ay - omega * vx) * dt; 
    next_x(5) = b_w; 

    // --- AGGIORNAMENTO DELLO JACOBIANO F ---
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    
    // Derivate di X e Y rispetto a Theta (usando th_mid)
    F(0, 2) = (-vx * std::sin(th_mid) - vy * std::cos(th_mid)) * dt;
    F(1, 2) = ( vx * std::cos(th_mid) - vy * std::sin(th_mid)) * dt;
    
    // Derivate di X e Y rispetto alle velocità (usando th_mid)
    F(0, 3) = std::cos(th_mid) * dt; 
    F(0, 4) = -std::sin(th_mid) * dt;
    F(1, 3) = std::sin(th_mid) * dt; 
    F(1, 4) = std::cos(th_mid) * dt;

    // Derivata di Theta rispetto al bias
    F(2, 5) = -dt; 

    // NOVITÀ: Derivate incrociate di X e Y rispetto al bias!
    // Siccome th_mid dipende da b_w, applichiamo la chain rule: d(th_mid)/d(b_w) = -dt/2
    F(0, 5) = F(0, 2) * (-dt / 2.0);
    F(1, 5) = F(1, 2) * (-dt / 2.0);
    
    // Derivate per l'aggiornamento delle velocità
    F(3, 4) = omega * dt;  
    F(3, 5) = -vy * dt; // Il bias influenza l'accelerazione di Coriolis calcolata
    F(4, 3) = -omega * dt; 
    F(4, 5) = vx * dt;

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