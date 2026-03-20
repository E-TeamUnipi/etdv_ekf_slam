#pragma once

#include <Eigen/Dense>
#include <vector>

class EKF_SLAM {
public:
    EKF_SLAM();

    void predict(double v, double vy, double omega, double dt);
    // Nota: rimosso l'array di IDs, l'EKF deve arrangiarsi con la sua logica!
    void correct(const Eigen::VectorXd& ranges, const Eigen::VectorXd& bearings);

    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }
    int numLandmarks() const { return n_landmarks_; }

    Eigen::Vector2d getLandmarkPosition(int index) const;
    
    void setProcessNoise(double noise_v, double noise_omega) {
        Q_(0,0) = noise_v;
        Q_(1,1) = noise_omega;
    }
    
    void setMeasurementNoise(double noise_range, double noise_bearing) {
        R_(0,0) = noise_range;
        R_(1,1) = noise_bearing;
    }
    
    void setAssociationThreshold(double threshold) {
        mahalanobis_threshold_ = threshold;
    }

private:
    Eigen::VectorXd x_;  
    Eigen::MatrixXd P_;  

    int n_landmarks_;

    Eigen::Matrix2d Q_;  
    Eigen::Matrix2d R_;  
    
    double mahalanobis_threshold_;

    int addLandmark(double range, double bearing);
    int findAssociation(double range, double bearing, const std::vector<int>& skip_ids);
    double normalizeAngle(double a);
};