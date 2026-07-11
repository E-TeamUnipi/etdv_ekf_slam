#include "ekf_slam/EKF_Pose.h"
#include <cmath>

EKFPose::EKFPose() {
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    R_ = Eigen::MatrixXd::Identity(1, 1) * 0.01;
}   

void EKFPose::setProcessNoise(double nv, double nvy, double nw) {
    Q_.diagonal() << 1e-4, 1e-4, 1e-4, nv*nv, nvy*nvy, nw*nw;
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

void EKFPose::correctPosition(double map_x, double map_y, double local_x, double local_y) {
    double curr_x = x_(0);
    double curr_y = x_(1);
    double curr_th = x_(2);

    // --- 1. MODELLO DI OSSERVAZIONE ---
    // Differenza tra il cono e la vettura nel frame globale
    double dx = map_x - curr_x;
    double dy = map_y - curr_y;

    // Dove "dovremmo" vedere il cono nel frame del LiDAR in base alla stima attuale?
    double expected_local_x = dx * std::cos(curr_th) + dy * std::sin(curr_th);
    double expected_local_y = -dx * std::sin(curr_th) + dy * std::cos(curr_th);

    // Vettore dell'innovazione z - h(x) (Errore tra misura reale e misura attesa)
    Eigen::VectorXd y_mat(2);
    y_mat(0) = local_x - expected_local_x;
    y_mat(1) = local_y - expected_local_y;

    // --- 2. JACOBIANO DELL'OSSERVAZIONE (H) ---
    // Derivate parziali di expected_local_x/y rispetto a [x, y, theta]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
    
    // Rispetto a X
    H(0, 0) = -std::cos(curr_th);
    H(1, 0) = std::sin(curr_th);
    
    // Rispetto a Y
    H(0, 1) = -std::sin(curr_th);
    H(1, 1) = -std::cos(curr_th);
    
    // Rispetto a Theta
    H(0, 2) = -dx * std::sin(curr_th) + dy * std::cos(curr_th);
    H(1, 2) = -dx * std::cos(curr_th) - dy * std::sin(curr_th);

    // --- 3. UPDATE KALMAN ---
    Eigen::MatrixXd R_pos = Eigen::MatrixXd::Identity(2, 2) * 0.1; // Aumentato leggermente per stabilità
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_pos;

    // Gating di Mahalanobis
    if (y_mat.transpose() * S.inverse() * y_mat > 25.0) return; 

    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // Aggiornamento Stato
    x_ = x_ + K * y_mat;
    
    // Normalizziamo l'angolo dopo l'update per evitare salti
    x_(2) = normalizeAngle(x_(2));

    // Joseph Form per la Covarianza
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R_pos * K.transpose();
}