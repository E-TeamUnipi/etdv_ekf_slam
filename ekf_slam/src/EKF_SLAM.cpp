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

EKF_SLAM::EKF_SLAM() {
    x_ = Eigen::VectorXd::Zero(3);
    P_ = Eigen::MatrixXd::Identity(3,3) * 1e-3;
    
    Q_ = Eigen::Matrix2d::Identity();
    Q_(0,0) = 0.5;  
    Q_(1,1) = 0.5;
    
    R_ = Eigen::Matrix2d::Identity() * 0.1;
    n_landmarks_ = 0;
    
    mahalanobis_threshold_ = 9.0; // Valore statistico per il loop closure
}

double EKF_SLAM::normalizeAngle(double a) {
    while (a >  PI_D) a -= 2.0 * PI_D;
    while (a < -PI_D) a += 2.0 * PI_D;
    return a;
}

void EKF_SLAM::predict(double v, double omega, double dt) {
    if (std::isnan(v) || std::isnan(omega) || std::isinf(v) || std::isinf(omega)) return;
    
    double theta = x_(2);
    double s = std::sin(theta);
    double c = std::cos(theta);

    if (std::abs(omega) > 1e-4) {
        x_(0) += (v / omega) * (std::sin(theta + omega * dt) - s);
        x_(1) += (v / omega) * (-std::cos(theta + omega * dt) + c);
        x_(2) += omega * dt;
    } else {
        x_(0) += v * std::cos(theta + omega * dt * 0.5) * dt;
        x_(1) += v * std::sin(theta + omega * dt * 0.5) * dt;
        x_(2) += omega * dt;
    }
    x_(2) = normalizeAngle(x_(2));

    int n = x_.size();
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
    if (std::abs(omega) > 1e-4) {
        F(0,2) = (v / omega) * (std::cos(theta + omega * dt) - c);
        F(1,2) = (v / omega) * (std::sin(theta + omega * dt) - s);
    } else {
        F(0,2) = -v * s * dt; 
        F(1,2) =  v * c * dt;
    }

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(n, 2);
    W(0,0) = c * dt; W(1,0) = s * dt; W(2,1) = dt;

    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
    M(0,0) = Q_(0,0) + 0.1 * std::abs(v); 
    M(1,1) = Q_(1,1) + 0.1 * std::abs(omega);

    Eigen::MatrixXd Qk = W * M * W.transpose();
    
    // Iniezione di incertezza costante per non far "addormentare" il filtro
    Qk(0,0) += 1e-3; 
    Qk(1,1) += 1e-3; 
    Qk(2,2) += 1e-4; 

    P_ = F * P_ * F.transpose() + Qk;
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

    Eigen::MatrixXd P_cross = Eigen::MatrixXd::Zero(old_size, 2);
    if (old_size >= 3) {
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

    for (int id = 0; id < n_landmarks_; ++id) {
        if (std::find(skip_ids.begin(), skip_ids.end(), id) != skip_ids.end()) continue;

        int offset = 3 + id * 2;
        double dx = x_(offset) - x_(0);
        double dy = x_(offset+1) - x_(1);
        double q = dx*dx + dy*dy;

        if (q < 1e-9) continue;

        double pred_range = std::sqrt(q);
        double pred_bearing = normalizeAngle(std::atan2(dy, dx) - x_(2));

        Eigen::Vector2d innov(range - pred_range, normalizeAngle(bearing - pred_bearing));

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, N);
        H(0,0) = -dx / pred_range; H(0,1) = -dy / pred_range; H(0,2) = 0.0;
        H(1,0) =  dy / q;          H(1,1) = -dx / q;          H(1,2) = -1.0;
        H(0,offset)   =  dx / pred_range; H(0,offset+1) =  dy / pred_range;
        H(1,offset)   = -dy / q;          H(1,offset+1) =  dx / q;

        Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
        
        // Mahalanobis Distance
        double mahalanobis_dist = innov.transpose() * S.inverse() * innov;
        
        // Check fisico (Euclideo) di sicurezza massima
        double meas_x = range * std::cos(bearing);
        double meas_y = range * std::sin(bearing);
        double theta = x_(2);
        double pred_x =  dx * std::cos(theta) + dy * std::sin(theta);
        double pred_y = -dx * std::sin(theta) + dy * std::cos(theta);
        double eucl_dist = std::hypot(meas_x - pred_x, meas_y - pred_y);

        if (mahalanobis_dist < min_dist && mahalanobis_dist < mahalanobis_threshold_ && eucl_dist < 4.0) {
            min_dist = mahalanobis_dist;
            best_id = id;
        }
    }
    return best_id;
}

void EKF_SLAM::correct(const Eigen::VectorXd& ranges, const Eigen::VectorXd& bearings) {
    if (ranges.size() != bearings.size()) return;

    std::vector<Measurement> meas_list;
    for (int i = 0; i < ranges.size(); ++i) {
        if (std::isnan(ranges(i)) || std::isnan(bearings(i))) continue;
        if (ranges(i) < 0.1 || ranges(i) > 30.0) continue;
        meas_list.push_back({ranges(i), bearings(i)});
    }

    // Ordinamento magico: Prima i coni vicini, poi quelli lontani (Previene il drift in curva)
    std::sort(meas_list.begin(), meas_list.end(), [](const Measurement& a, const Measurement& b) {
        return a.range < b.range;
    });

    std::vector<int> updated_landmarks;

    for (const auto& m : meas_list) {
        int internal_id = findAssociation(m.range, m.bearing, updated_landmarks);

        if (internal_id == -1) {
            internal_id = addLandmark(m.range, m.bearing);
            updated_landmarks.push_back(internal_id);
            continue; 
        }

        updated_landmarks.push_back(internal_id);

        int offset = 3 + internal_id * 2;
        double dx = x_(offset)   - x_(0);
        double dy = x_(offset+1) - x_(1);
        double q = dx*dx + dy*dy;

        if (q < 1e-9) continue;

        double pred_range   = std::sqrt(q);
        double pred_bearing = normalizeAngle(std::atan2(dy, dx) - x_(2));

        Eigen::Vector2d y(m.range - pred_range, normalizeAngle(m.bearing - pred_bearing));

        int N = x_.size();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, N);

        H(0,0) = -dx / pred_range; H(0,1) = -dy / pred_range; H(0,2) = 0.0;
        H(1,0) =  dy / q;          H(1,1) = -dx / q;          H(1,2) = -1.0;
        H(0,offset)   =  dx / pred_range; H(0,offset+1) =  dy / pred_range;
        H(1,offset)   = -dy / q;          H(1,offset+1) =  dx / q;

        Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
        
        if (std::abs(S.determinant()) < 1e-12) continue;

        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        x_ = x_ + K * y;
        x_(2) = normalizeAngle(x_(2));
        
        // Forma di Joseph sicura (Previene il crash della covarianza)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);
        Eigen::MatrixXd I_KH = I - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose(); 
        P_ = (P_ + P_.transpose()) * 0.5;
    }
}
