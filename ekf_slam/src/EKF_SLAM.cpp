#include "ekf_slam/EKF_SLAM.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>

static constexpr double PI_D = std::acos(-1.0);

struct Measurement {
    double range;
    double bearing;
};

// ---------------------------------------------------------------------------
// Helper: calcola H*P sfruttando la struttura sparsa di H.
//
// H è una matrice 2×N con solo 5 colonne non-zero:
//   col 0,1,2  → parte robot
//   col offset, offset+1 → parte landmark
//
// Invece di moltiplicare l'intera matrice 2×N · N×N (O(N²)),
// estraiamo le 5 righe rilevanti di P e moltiplichiamo solo quelle (O(N)).
//
// h_rob  = [h00, h01, h02]   (riga 0 e 1 per le colonne robot)
// h_lmk  = [h0o, h0o1; h1o, h1o1]   (colonne landmark)
// Ritorna una matrice 2×N.
// ---------------------------------------------------------------------------
static Eigen::Matrix<double,2,Eigen::Dynamic>
sparseHP(const Eigen::MatrixXd& P,
         double h00, double h01,          // H(0, 0..1) — H(0,2)=0
         double h10, double h11,          // H(1, 0..1)
         double h0o, double h0o1,         // H(0, offset), H(0, offset+1)
         double h1o, double h1o1,         // H(1, offset), H(1, offset+1)
         double h12,                      // H(1, 2) = -1
         int offset)
{
    int N = P.cols();

    // HP = H * P  →  riga i di HP = somma_j H(i,j) * riga j di P
    // Solo j ∈ {0, 1, 2, offset, offset+1} sono non-zero.

    Eigen::Matrix<double,2,Eigen::Dynamic> HP(2, N);

    HP.row(0) = h00  * P.row(0)
              + h01  * P.row(1)
              // h02 = 0, skip
              + h0o  * P.row(offset)
              + h0o1 * P.row(offset + 1);

    HP.row(1) = h10  * P.row(0)
              + h11  * P.row(1)
              + h12  * P.row(2)          // h12 = -1
              + h1o  * P.row(offset)
              + h1o1 * P.row(offset + 1);

    return HP;
}

// ---------------------------------------------------------------------------
// Helper: calcola S = HP * H^T + R  sfruttando ancora la sparsità.
// HP è già 2×N, serve solo estrarre le colonne {0,1,2,offset,offset+1}.
// ---------------------------------------------------------------------------
static Eigen::Matrix2d
sparseS(const Eigen::Matrix<double,2,Eigen::Dynamic>& HP,
        double h00, double h01,
        double h10, double h11,
        double h0o, double h0o1,
        double h1o, double h1o1,
        double h12,
        int offset,
        const Eigen::Matrix2d& R)
{
    // S(i,j) = sum_k HP(i,k) * H(j,k)  + R(i,j)
    // solo k ∈ {0,1,2,offset,offset+1}

    Eigen::Matrix2d S = R;

    // row 0 of H (h02=0)
    S(0,0) += HP(0,0)*h00 + HP(0,1)*h01 + HP(0,offset)*h0o + HP(0,offset+1)*h0o1;
    S(0,1) += HP(0,0)*h10 + HP(0,1)*h11 + HP(0,2)*h12 + HP(0,offset)*h1o + HP(0,offset+1)*h1o1;
    S(1,0)  = S(0,1);
    S(1,1) += HP(1,0)*h10 + HP(1,1)*h11 + HP(1,2)*h12 + HP(1,offset)*h1o + HP(1,offset+1)*h1o1;

    return S;
}

// ===========================================================================

EKF_SLAM::EKF_SLAM() {
    x_ = Eigen::VectorXd::Zero(3);
    P_ = Eigen::MatrixXd::Identity(3,3) * 1e-3;
    
    Q_ = Eigen::Matrix2d::Identity();
    Q_(0,0) = 0.5;  
    Q_(1,1) = 0.5;
    
    R_ = Eigen::Matrix2d::Identity() * 0.1;
    n_landmarks_ = 0;
    
    mahalanobis_threshold_ = 9.0; 
}

double EKF_SLAM::normalizeAngle(double a) {
    while (a >  PI_D) a -= 2.0 * PI_D;
    while (a < -PI_D) a += 2.0 * PI_D;
    return a;
}

void EKF_SLAM::predict(double v, double vy, double omega, double dt) {
    if (std::isnan(v) || std::isnan(vy) || std::isnan(omega) ||
        std::isinf(v) || std::isinf(vy) || std::isinf(omega)) return;
    
    double theta = x_(2);
    double s = std::sin(theta);
    double c = std::cos(theta);

    // Modello cinematico con velocita' laterale vy (nel frame del robot).
    // Le coordinate globali si aggiornano ruotando (v, vy) nel frame map:
    //   dx_map = v*cos(theta) - vy*sin(theta)
    //   dy_map = v*sin(theta) + vy*cos(theta)
    // Il termine con omega e' integrato esattamente per v (curva costante).
    // vy e' trattato con integrazione lineare (tipicamente piccolo).
    if (std::abs(omega) > 1e-4) {
        x_(0) += (v / omega) * (std::sin(theta + omega * dt) - s)
               + vy * dt * (-s);  // contributo laterale approssimato
        x_(1) += (v / omega) * (-std::cos(theta + omega * dt) + c)
               + vy * dt * c;
        x_(2) += omega * dt;
    } else {
        double theta_mid = theta + omega * dt * 0.5;
        x_(0) += (v * std::cos(theta_mid) - vy * std::sin(theta_mid)) * dt;
        x_(1) += (v * std::sin(theta_mid) + vy * std::cos(theta_mid)) * dt;
        x_(2) += omega * dt;
    }
    x_(2) = normalizeAngle(x_(2));

    // --- predict: F è identità + due elementi in colonna 2 → O(N) ---
    // Invece di costruire F densa N×N e fare F*P*F^T (O(N²) denso),
    // sfruttiamo il fatto che F = I + delta_col2 * e_2^T:
    //   F*P*F^T = P  +  d * P.row(2)  (aggiunta a colonne)
    //              +  P.col(2) * d^T   (aggiunta a righe)
    //              +  (d^T * P.col(2)) * d * d^T  (termine scalare)
    // dove d = [0, 0, 0, ..., F(0,2), F(1,2), 0, ...]^T (solo righe 0 e 1 non-zero)

    // Jacobiano F: derivate parziali di x rispetto a theta.
    // Con vy: d(x)/d(theta) += -vy*cos(theta)*dt, d(y)/d(theta) += -vy*sin(theta)*dt
    double f02, f12;
    if (std::abs(omega) > 1e-4) {
        f02 = (v / omega) * (std::cos(theta + omega * dt) - c) - vy * s * dt;
        f12 = (v / omega) * (std::sin(theta + omega * dt) - s) + vy * c * dt;
    } else {
        double theta_mid = theta + omega * dt * 0.5;
        f02 = -v * std::sin(theta_mid) * dt - vy * std::cos(theta_mid) * dt;
        f12 =  v * std::cos(theta_mid) * dt - vy * std::sin(theta_mid) * dt;
    }

    // P ← F * P * F^T  usando rank-2 update (O(N²) ma con costante molto minore)
    // F*P: modifica solo righe 0 e 1
    //   (F*P).row(0) = P.row(0) + f02 * P.row(2)
    //   (F*P).row(1) = P.row(1) + f12 * P.row(2)
    //   altre righe invariate
    // Poi (F*P)*F^T: modifica solo colonne 0 e 1
    //   result.col(0) += f02 * result.col(2)   (già aggiornata)
    //   result.col(1) += f12 * result.col(2)

    Eigen::VectorXd p_col2 = P_.col(2);   // salva prima di modificare

    P_.row(0) += f02 * P_.row(2);
    P_.row(1) += f12 * P_.row(2);

    // ora aggiungi alle colonne (trasposto)
    P_.col(0) += f02 * p_col2;
    P_.col(1) += f12 * p_col2;
    // elemento (0,0): doppio contributo f02² * P_orig(2,2) — già incluso nei due passi sopra
    // ma p_col2 era la colonna originale, quindi è corretto.

    // Qk: W è sparsa (3 righe non-zero su N), Qk è un rank-2 update O(N)
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(x_.size(), 2);
    W(0,0) = c * dt; W(1,0) = s * dt; W(2,1) = dt;

    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
    M(0,0) = Q_(0,0) + 0.1 * std::abs(v); 
    M(1,1) = Q_(1,1) + 0.1 * std::abs(omega);

    // Qk = W * M * W^T — solo 3 righe di W sono non-zero → O(N)
    Eigen::MatrixXd Qk = W * M * W.transpose();
    
    Qk(0,0) += 1e-2; 
    Qk(1,1) += 1e-2; 
    Qk(2,2) += 1e-2; // rilassa sicurezza sull'odometria

    P_ += Qk;
    P_ = (P_ + P_.transpose()) * 0.5;
}

int EKF_SLAM::addLandmark(double range, double bearing) {
    double theta = x_(2);
    double lx = x_(0) + range * std::cos(theta + bearing);
    double ly = x_(1) + range * std::sin(theta + bearing);

    int old_size = x_.size();
    if (P_.rows() != old_size || P_.cols() != old_size) {
        Eigen::MatrixXd P_fixed = Eigen::MatrixXd::Identity(old_size, old_size) * 1.0;
        P_fixed.block(0,0, P_.rows(), P_.cols()) = P_; 
        P_ = P_fixed;
    }

    int new_size = old_size + 2;
    Eigen::VectorXd new_x(new_size);
    new_x.head(old_size) = x_;
    new_x(old_size) = lx;
    new_x(old_size + 1) = ly;
    x_ = new_x;

    Eigen::MatrixXd new_P = Eigen::MatrixXd::Zero(new_size, new_size);
    new_P.topLeftCorner(old_size, old_size) = P_;

    double c_tb = std::cos(theta + bearing);
    double s_tb = std::sin(theta + bearing);

    Eigen::MatrixXd G_robot(2, 3);
    G_robot << 1.0, 0.0, -range * s_tb,
               0.0, 1.0,  range * c_tb;

    Eigen::Matrix2d G_meas;
    G_meas << c_tb, -range * s_tb,
              s_tb,  range * c_tb;

    Eigen::Matrix2d P_landmark = G_meas * R_ * G_meas.transpose();
    if (P_.rows() >= 3) {
         P_landmark += G_robot * P_.topLeftCorner(3,3) * G_robot.transpose();
    }

    new_P.block<2,2>(new_size-2, new_size-2) = P_landmark;

    // FIX B2: propaga le cross-covarianze usando tutte le colonne di P_old,
    // non solo le prime 3. P_cross(old_size×2) = P_old * G_robot^T
    // dove G_robot opera sulle prime 3 righe dello stato.
    // Equivalente a: P_cross = P_old.leftCols(3) * G_robot^T  [corretto]
    // ma ora usiamo P_old completa per includere anche i landmark già mappati:
    //   P_cross_full = [ P_old.leftCols(3) * G_robot^T ]   (solo le prime 3 colonne contano)
    Eigen::MatrixXd P_cross = Eigen::MatrixXd::Zero(old_size, 2);
    if (old_size >= 3) {
        // G_robot è 2×3, agisce sulle colonne {0,1,2} → P_old * G_robot^T
        // è equivalente a usare le colonne 0,1,2 di P_old
        P_cross = P_.leftCols(3) * G_robot.transpose();
    }
    
    new_P.block(0, new_size-2, old_size, 2) = P_cross;
    new_P.block(new_size-2, 0, 2, old_size) = P_cross.transpose();

    P_ = new_P;

    int internal_id = n_landmarks_;
    n_landmarks_++;
    return internal_id;
}

Eigen::Vector2d EKF_SLAM::getLandmarkPosition(int index) const {
    if (index < 0 || index >= n_landmarks_) return Eigen::Vector2d::Zero();
    int offset = 3 + index*2;
    return Eigen::Vector2d(x_(offset), x_(offset+1));
}

int EKF_SLAM::findAssociation(double range, double bearing, const std::vector<int>& skip_ids) {
    if (n_landmarks_ == 0) return -1;

    double min_dist = std::numeric_limits<double>::infinity();
    int best_id = -1;
    int N = x_.size();

    // Pre-calcola le misure in coordinate sensore per il gate euclideo
    double meas_x = range * std::cos(bearing);
    double meas_y = range * std::sin(bearing);
    double cos_theta = std::cos(x_(2));
    double sin_theta = std::sin(x_(2));

    for (int id = 0; id < n_landmarks_; ++id) {
        if (std::find(skip_ids.begin(), skip_ids.end(), id) != skip_ids.end()) continue;

        int offset = 3 + id * 2;
        double dx = x_(offset)   - x_(0);
        double dy = x_(offset+1) - x_(1);
        double q  = dx*dx + dy*dy;

        if (q < 1e-9) continue;

        double pred_range   = std::sqrt(q);
        double pred_bearing = normalizeAngle(std::atan2(dy, dx) - x_(2));

        // Gate euclideo veloce — scarta subito i landmark lontani prima di
        // calcolare la Mahalanobis (evita H*P su candidati improbabili)
        double pred_x =  dx * cos_theta + dy * sin_theta;
        double pred_y = -dx * sin_theta + dy * cos_theta;
        double eucl_dist = std::hypot(meas_x - pred_x, meas_y - pred_y);
        // FIX 3: gate euclideo a 2.5m (ripristinato: a 6m associava il cono successivo)
        if (eucl_dist >= 2.5) continue;

        // Coefficienti non-zero di H (matrice 2×N sparsa)
        double h00 = -dx / pred_range,  h01 = -dy / pred_range;
        double h10 =  dy / q,           h11 = -dx / q,          h12 = -1.0;
        double h0o =  dx / pred_range,  h0o1 =  dy / pred_range;
        double h1o = -dy / q,           h1o1 =  dx / q;

        // --- OPT 1: H*P sparse — O(N) invece di O(N²) ---
        auto HP = sparseHP(P_, h00, h01, h10, h11, h0o, h0o1, h1o, h1o1, h12, offset);

        // --- OPT 1b: S = HP*H^T + R sparse ---
        Eigen::Matrix2d S = sparseS(HP, h00, h01, h10, h11, h0o, h0o1, h1o, h1o1, h12, offset, R_);

        // --- OPT 2: LDLT invece di inverse() — più stabile e veloce su 2×2 ---
        Eigen::Vector2d innov(range - pred_range, normalizeAngle(bearing - pred_bearing));
        Eigen::Vector2d S_inv_innov = S.ldlt().solve(innov);
        double mahalanobis_dist = innov.dot(S_inv_innov);

        if (mahalanobis_dist < min_dist && mahalanobis_dist < mahalanobis_threshold_) {
            min_dist = mahalanobis_dist;
            best_id  = id;
        }
    }
    return best_id;
}

void EKF_SLAM::correct(const Eigen::VectorXd& ranges, const Eigen::VectorXd& bearings) {
    if (ranges.size() != bearings.size()) return;

    std::vector<Measurement> meas_list;
    for (int i = 0; i < ranges.size(); ++i) {
        if (std::isnan(ranges(i)) || std::isnan(bearings(i))) continue;
        
        // --- FIX 1: MIOPIA TATTICA ---
        // I coni oltre i 12 metri hanno un errore trasversale letale a causa del drift di theta.
        if (ranges(i) < 0.1 || ranges(i) > 20.0) continue; 
        meas_list.push_back({ranges(i), bearings(i)});
    }

    std::sort(meas_list.begin(), meas_list.end(), [](const Measurement& a, const Measurement& b) {
        return a.range < b.range;
    });

    std::vector<int> updated_landmarks;

    for (const auto& m : meas_list) {
        int internal_id = findAssociation(m.range, m.bearing, updated_landmarks);

        if (internal_id == -1) {
            // --- FIX 2: SCUDO ANTI-DUPLICAZIONE (Il salvavita) ---
            bool is_ghost = false;
            double meas_x = m.range * std::cos(m.bearing);
            double meas_y = m.range * std::sin(m.bearing);
            double theta  = x_(2);
            double global_x = x_(0) + meas_x * std::cos(theta) - meas_y * std::sin(theta);
            double global_y = x_(1) + meas_x * std::sin(theta) + meas_y * std::cos(theta);

            for (int j = 0; j < n_landmarks_; ++j) {
                double lx = x_(3 + j*2);
                double ly = x_(3 + j*2 + 1);
                if (std::hypot(global_x - lx, global_y - ly) < 1.5) {
                    is_ghost = true; 
                    break;
                }
            }

            if (!is_ghost) {
                internal_id = addLandmark(m.range, m.bearing);
                updated_landmarks.push_back(internal_id);
            }
            continue; 
        }

        updated_landmarks.push_back(internal_id);

        int offset = 3 + internal_id * 2;
        double dx = x_(offset)   - x_(0);
        double dy = x_(offset+1) - x_(1);
        double q  = dx*dx + dy*dy;

        if (q < 1e-9) continue;

        double pred_range   = std::sqrt(q);
        double pred_bearing = normalizeAngle(std::atan2(dy, dx) - x_(2));

        Eigen::Vector2d innov(m.range - pred_range, normalizeAngle(m.bearing - pred_bearing));

        int N = x_.size();

        // Coefficienti non-zero di H
        double h00 = -dx / pred_range,  h01 = -dy / pred_range;
        double h10 =  dy / q,           h11 = -dx / q,          h12 = -1.0;
        double h0o =  dx / pred_range,  h0o1 =  dy / pred_range;
        double h1o = -dy / q,           h1o1 =  dx / q;

        // --- OPT 1: H*P sparse O(N) ---
        auto HP = sparseHP(P_, h00, h01, h10, h11, h0o, h0o1, h1o, h1o1, h12, offset);

        // --- OPT 1b: S sparse ---
        Eigen::Matrix2d S = sparseS(HP, h00, h01, h10, h11, h0o, h0o1, h1o, h1o1, h12, offset, R_);

        // --- OPT 2: LDLT solve ---
        Eigen::LDLT<Eigen::Matrix2d> S_ldlt(S);
        if (S_ldlt.info() != Eigen::Success) continue;

        // K = P * H^T * S^{-1}  →  K = (S^{-T} * H * P)^T = (S^{-1} * HP)^T
        // HP è 2×N, S è 2×2 → K = S^{-1} * HP  poi trasposta → K è N×2
        Eigen::MatrixXd K = S_ldlt.solve(HP).transpose();   // N×2

        x_ += K * innov;
        x_(2) = normalizeAngle(x_(2));

        // --- OPT 3: Joseph form per P ---
        // P ← (I - K*H) * P * (I - K*H)^T + K * R * K^T
        // Garantisce semidefinita positività senza simmetrizzazione esplicita.
        //
        // (I - K*H) è N×N ma K*H ha rango 2 → costruiamo solo il prodotto
        // sfruttando ancora la sparsità di H:
        //   K*H * v  =  K * (H*v)  dove H*v usa solo 5 elementi di v

        // Costruiamo KH = K * H  (N×N, rango 2) — ma non esplicitamente:
        // usiamo  (I - KH)*P = P - K*(HP)  e poi il termine simmetrico
        Eigen::MatrixXd IKH_P = P_ - K * HP;   // N×N, O(N²) inevitabile per P update

        // P ← (I-KH)*P*(I-KH)^T + K*R*K^T
        // (I-KH)^T = I - H^T*K^T  → IKH_P * (I - H^T*K^T) = IKH_P - IKH_P*H^T*K^T
        // IKH_P * H^T  →  sparso: H^T ha colonne non-zero solo in 0,1,2,offset,offset+1
        // (IKH_P * H^T) è N×2 e si calcola O(N):
        Eigen::MatrixXd IKH_P_HT(N, 2);
        IKH_P_HT.col(0) = h00 * IKH_P.col(0) + h01 * IKH_P.col(1)
                         + h0o * IKH_P.col(offset) + h0o1 * IKH_P.col(offset+1);
        IKH_P_HT.col(1) = h10 * IKH_P.col(0) + h11 * IKH_P.col(1)
                         + h12 * IKH_P.col(2)
                         + h1o * IKH_P.col(offset) + h1o1 * IKH_P.col(offset+1);

        P_ = IKH_P - IKH_P_HT * K.transpose()   // (I-KH)*P*(I-KH)^T
           + K * R_ * K.transpose();              // + K*R*K^T

        // Simmetrizzazione leggera per errori di floating point residui
        P_ = (P_ + P_.transpose()) * 0.5;
    }
}
