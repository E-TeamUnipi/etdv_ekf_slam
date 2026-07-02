#include "ekf_slam/EKF_Pose.h"
#include <cmath>

EKFPose::EKFPose() {
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01; // Default
    R_ = Eigen::MatrixXd::Identity(1, 1) * 0.01; // Default;
}   

void EKFPose::setProcessNoise(double nv, double nvy, double nw) {
    Q_.diagonal() << nv*nv, nv*nv, nw*nw, nvy*nvy, nvy*nvy, nw*nw;
}

double EKFPose::normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void EKFPose::predict(double ax, double ay, double dt) {
    double th = x_(2);
    double vx = x_(3);
    double vy = x_(4);

    double omega = x_(5);

    // 1. Stato aggiornato (Modello non-lineare con Coriolis)
    Eigen::VectorXd next_x(6);
    next_x(0) = x_(0) + (vx * std::cos(th) - vy * std::sin(th)) * dt;
    next_x(1) = x_(1) + (vx * std::sin(th) + vy * std::cos(th)) * dt;
    next_x(2) = normalizeAngle(x_(2) + omega * dt);
    next_x(3) = vx + (ax + omega * vy) * dt; // Compensazione Coriolis
    next_x(4) = vy + (ay - omega * vx) * dt; // Compensazione Coriolis
    next_x(5) = omega;

    // 2. Jacobiano F (6x6)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 2) = (-vx * std::sin(th) - vy * std::cos(th)) * dt;
    F(0, 3) = std::cos(th) * dt; F(0, 4) = -std::sin(th) * dt;
    F(1, 2) = ( vx * std::cos(th) - vy * std::sin(th)) * dt;
    F(1, 3) = std::sin(th) * dt; F(1, 4) = std::cos(th) * dt;
    F(2, 5) = dt;
    F(3, 4) = omega * dt;  F(3, 5) = vy * dt;
    F(4, 3) = -omega * dt; F(4, 5) = -vx * dt;

    // 3. Propagazione P = F*P*F' + Q
    P_ = F * P_ * F.transpose() + Q_;
    x_ = next_x;
}

void EKFPose::correctGyro(double z_gyro, double& out_y, double& out_k) {
    // 1. Matrice di misura H (1x6)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
    H(0, 5) = 1.0; 

    // 2. Creiamo il vettore di misura (z) come matrice 1x1
    Eigen::MatrixXd z(1, 1);
    z(0, 0) = z_gyro;

    // 3. Innovazione (y) come matrice
    Eigen::MatrixXd y_mat = z - H * x_; 
    double y = y_mat(0, 0); // Estraiamo lo scalare per comodità

    // 4. Covarianza dell'Innovazione (S) come matrice
    // R_ ora deve essere una matrice 1x1
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // 5. Guadagno di Kalman (K = PH' * S.inverse())
    // Nota: per matrici 1x1, .inverse() è matematicamente equivalente a 1/S
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // ESPORTIAMO I VALORI AL NODO
    out_y = y;
    out_k = K(5, 0);

    // 6. Aggiornamento Stato
    x_ = x_ + K * y_mat;

    // 7. Aggiornamento Covarianza
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P_ = (I - K * H) * P_;

    // 8. Log (estrai il valore dal primo elemento della matrice y per il log)
    logDiagnostic(0.0, 0.0, y); 
}

void EKFPose::logDiagnostic(double ts, double res_ax, double res_ay) {
    diagnostics.push_back({ts, res_ax, res_ay});
    if (diagnostics.size() > 5000) diagnostics.erase(diagnostics.begin());
}