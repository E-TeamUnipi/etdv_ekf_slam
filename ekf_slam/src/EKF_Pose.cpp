#include "ekf_slam/EKF_Pose.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

EKFPose::EKFPose() {
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    R_ = Eigen::MatrixXd::Identity(2, 2) * 0.01;
}   

void EKFPose::setProcessNoise(double x, double y, double yaw, double nv, double nvy, double nw, double nlx, double nly) {
    Q_.diagonal() << x*x, y*y, yaw*yaw, nv*nv, nvy*nvy, nw*nw;
    R_.diagonal() << nlx*nlx, nly*nly; 
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

    Eigen::VectorXd next_x = x_;
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

    int n = x_.size();

    if (n == 6) {
        // FASE 1: Nessun cono mappato. Il filtro si comporta come una localizzazione standard.
        P_ = F * P_ * F.transpose() + Q_;
    } else {
        // FASE 2: SLAM. La mappa contiene dei coni, dobbiamo usare i blocchi.
        int lm_dim = n - 6; // Dimensione occupata dai landmark (es. 2, 4, 6...)
        
        // 1. Aggiorna la covarianza del veicolo (blocco 6x6 in alto a sinistra)
        P_.block(0, 0, 6, 6) = F * P_.block(0, 0, 6, 6) * F.transpose() + Q_;
        
        // 2. Aggiorna la covarianza incrociata Veicolo-Mappa (blocco 6 x lm_dim in alto a destra)
        P_.block(0, 6, 6, lm_dim) = F * P_.block(0, 6, 6, lm_dim);
        
        // 3. Aggiorna la covarianza incrociata Mappa-Veicolo (blocco lm_dim x 6 in basso a sinistra)
        // È semplicemente la trasposta del blocco calcolato al passo precedente
        P_.block(6, 0, lm_dim, 6) = P_.block(0, 6, 6, lm_dim).transpose(); 
        
        // 4. Il blocco Mappa-Mappa (in basso a destra) RESTA INVARIATO
        // Matematicamente sarebbe: P_mm = I * P_mm * I^T = P_mm
    }

    x_ = next_x;
}

void EKFPose::correctPosition(int matched_id, double local_x_meas, double local_y_meas) {
    
    int state_dim = x_.size(); // Dimensione dinamica dello stato

    // 1. Estrai la posa attuale del veicolo
    double x_v = x_(0);
    double y_v = x_(1);
    double theta_v = x_(2);

    // 2. Estrai la posizione globale del cono mappato dal vettore di stato
    int landmark_idx = 6 + 2 * matched_id;
    double m_x = x_(landmark_idx);
    double m_y = x_(landmark_idx + 1);

    // 3. Calcola l'osservazione predetta (Z_hat)
    // Ovvero: "Dove mi aspetterei di vedere questo cono rispetto alla macchina?"
    double dx = m_x - x_v;
    double dy = m_y - y_v;
    
    double c_th = std::cos(theta_v);
    double s_th = std::sin(theta_v);

    Eigen::Vector2d Z_hat;
    Z_hat(0) = dx * c_th + dy * s_th;
    Z_hat(1) = -dx * s_th + dy * c_th;

    // Vettore della misurazione reale (Z)
    Eigen::Vector2d Z_meas(local_x_meas, local_y_meas);
    
    // Calcolo dell'Innovazione (Errore di predizione)
    Eigen::Vector2d Y = Z_meas - Z_hat;

    // ==========================================
    // 4. JACOBIANI SPAZIATI (SPARSE) - Solo i blocchi utili!
    // ==========================================
    Eigen::Matrix<double, 2, 3> H_v;
    H_v << -c_th, -s_th, -dx * s_th + dy * c_th,
            s_th, -c_th, -dx * c_th - dy * s_th;

    Eigen::Matrix2d H_l;
    H_l << c_th, s_th,
          -s_th, c_th;

    // ==========================================
    // 5. AGGIORNAMENTO O(N) AD ALTISSIME PRESTAZIONI
    // ==========================================
    
    // 1. Calcolo di P * H^T bypassando gli zeri. 
    // Risultato diretto in una matrice dinamica (state_dim x 2)
    Eigen::MatrixXd PHt(state_dim, 2);
    PHt = P_.block(0, 0, state_dim, 3) * H_v.transpose() + 
          P_.block(0, landmark_idx, state_dim, 2) * H_l.transpose();

    // 2. Covarianza dell'Innovazione (S)
    // Estraiamo solo le righe di PHt relative alla vettura e al landmark corrente
    Eigen::Matrix2d S = H_v * PHt.block(0, 0, 3, 2) + 
                        H_l * PHt.block(landmark_idx, 0, 2, 2) + R_;

    // 3. Guadagno K
    Eigen::MatrixXd K = PHt * S.inverse();

    // 4. Aggiornamento Stato
    x_ = x_ + K * Y;
    x_(2) = normalizeAngle(x_(2));

    // 5. Aggiornamento P e forma di Joseph
    P_.noalias() -= K * S * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

void EKFPose::addNewLandmark(double local_x, double local_y) {
    int current_dim = x_.size();
    int new_dim = current_dim + 2;

    double x_v = x_(0);
    double y_v = x_(1);
    double theta_v = x_(2);

    double c_th = std::cos(theta_v);
    double s_th = std::sin(theta_v);

    // Proiezione Globale
    double map_x = x_v + local_x * c_th - local_y * s_th;
    double map_y = y_v + local_x * s_th + local_y * c_th;

    double dx = map_x - x_v;
    double dy = map_y - y_v;

    // Jacobiani
    Eigen::Matrix<double, 2, 3> G_X;
    G_X << 1.0, 0.0, -dy,
           0.0, 1.0,  dx;

    Eigen::Matrix2d G_z;
    G_z << c_th, -s_th,
           s_th,  c_th;

    // ==========================================
    // LA CHIAVE DELLA SLAM: Covarianza Incrociata
    // Moltiplicando il blocco (current_dim x 3) per G_X^T, 
    // leghiamo il nuovo cono a TUTTI gli stati esistenti!
    // ==========================================
    Eigen::MatrixXd P_cross = P_.block(0, 0, current_dim, 3) * G_X.transpose();

    Eigen::Matrix3d P_vv = P_.block(0, 0, 3, 3);
    Eigen::Matrix2d P_ll = G_X * P_vv * G_X.transpose() + G_z * R_ * G_z.transpose();

    x_.conservativeResize(new_dim);
    x_(current_dim) = map_x;
    x_(current_dim + 1) = map_y;

    P_.conservativeResize(new_dim, new_dim);
    P_.block(0, current_dim, current_dim, 2) = P_cross;               // Colonna
    P_.block(current_dim, 0, 2, current_dim) = P_cross.transpose();   // Riga
    P_.block(current_dim, current_dim, 2, 2) = P_ll;                  // Blocco cono
}

int EKFPose::dataAssociation(double local_x, double local_y, double mahalanobis_th) {
    int state_size = x_.size();
    if (state_size <= 6) return -1; // Nessun cono mappato

    int num_cones = (state_size - 6) / 2;
    int best_id = -1;
    double min_dist = mahalanobis_th; 
    double true_min_dist = 1e9;

    double x_v = x_(0);
    double y_v = x_(1);
    double theta_v = x_(2);
    double c_th = std::cos(theta_v);
    double s_th = std::sin(theta_v);
    
    Eigen::Vector2d Z_meas(local_x, local_y);

    for (int i = 0; i < num_cones; ++i) {
        int landmark_idx = 6 + 2 * i;
        double m_x = x_(landmark_idx);
        double m_y = x_(landmark_idx + 1);

        double dx = m_x - x_v;
        double dy = m_y - y_v;

        Eigen::Vector2d Z_hat;
        Z_hat(0) = dx * c_th + dy * s_th;
        Z_hat(1) = -dx * s_th + dy * c_th;

        Eigen::Vector2d Y = Z_meas - Z_hat;

        // gate di sicurezza per non comprendere coni troppo distanti
        if (Y.norm() > 1.5){
            continue;
        }

        // // Costruzione dinamica di H per esplorare l'incertezza
        // Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state_size);
        // H(0, 0) = -c_th; H(0, 1) = -s_th; H(0, 2) = -dx * s_th + dy * c_th;
        // H(1, 0) =  s_th; H(1, 1) = -c_th; H(1, 2) = -dx * c_th - dy * s_th;
        // H(0, landmark_idx) = c_th; H(0, landmark_idx + 1) = s_th;
        // H(1, landmark_idx) = -s_th; H(1, landmark_idx + 1) = c_th;

        // introduciamo il parcing delle matrici per alleggerire il carico computazionale
        // 3. CALCOLO OTTIMIZZATO A BLOCCHI O(1) 
        // Invece di allocare un'enorme matrice H piena di zeri, 
        // estraiamo solo le derivate e le covarianze che ci interessano.
        Eigen::Matrix<double, 2, 3> H_v;
        H_v << -c_th, -s_th, -dx * s_th + dy * c_th,
                s_th, -c_th, -dx * c_th - dy * s_th;

        Eigen::Matrix2d H_l;
        H_l << c_th, s_th,
              -s_th, c_th;

        Eigen::Matrix3d P_vv = P_.block(0, 0, 3, 3);
        Eigen::Matrix2d P_ll = P_.block(landmark_idx, landmark_idx, 2, 2);
        Eigen::Matrix<double, 3, 2> P_vl = P_.block(0, landmark_idx, 3, 2);
        Eigen::Matrix<double, 2, 3> P_lv = P_.block(landmark_idx, 0, 2, 3);

        Eigen::Matrix2d S = H_v * P_vv * H_v.transpose() +
                            H_v * P_vl * H_l.transpose() +
                            H_l * P_lv * H_v.transpose() +
                            H_l * P_ll * H_l.transpose() + R_;
        // Matrice dell'Innovazione S
        // Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
        // --- AGGIUNGI QUESTO "CUSCINETTO" ---
        // Inflazione del rumore per evitare l'overconfidence (es. 10cm di incertezza minima)
        // double inflation = 0.01; // 10cm di incertezza minima
        // S(0, 0) += inflation;
        // S(1, 1) += inflation;
        // ------------------------------------
        // Calcolo della Distanza di Mahalanobis
        double mahalanobis_dist = Y.transpose() * S.inverse() * Y;

        if (mahalanobis_dist < true_min_dist) true_min_dist = mahalanobis_dist;
        if (mahalanobis_dist < min_dist) {
            min_dist = mahalanobis_dist;
            best_id = i;
        }
    }

    double min_euclid = 1e9;
    int nearest_id = -1;
    for (int i = 0; i < num_cones; ++i) {
        int landmark_idx = 6 + 2 * i;
        double ddx = x_(landmark_idx)     - (x_v + local_x*c_th - local_y*s_th);
        double ddy = x_(landmark_idx + 1) - (y_v + local_x*s_th + local_y*c_th);
        double d = std::hypot(ddx, ddy);
        if (d < min_euclid) { min_euclid = d; nearest_id = i; }
    }
    // RCLCPP_INFO(rclcpp::get_logger("ekf"), "min_mahalanobis=%.2f (thresh=%.2f) | euclid_nearest=%.3f m (id=%d)",
    //             min_dist, mahalanobis_th, min_euclid, nearest_id);

    // RCLCPP_INFO(rclcpp::get_logger("ekf"), 
    // "true_min_mahal=%.2f (thresh=%.2f) | euclid=%.3f m (id=%d) | theta=%.3f",
    // true_min_dist, mahalanobis_th, min_euclid, nearest_id, theta_v);
    
    return best_id;
}
