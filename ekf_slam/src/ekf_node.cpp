#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp> // Nuovo standard per Pacsim
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mutex>
#include <memory>
#include <vector>
#include <cmath>

#include "ekf_slam/EKF_SLAM.h"

using std::placeholders::_1;

class EKFNode : public rclcpp::Node {
public:
  EKFNode() : Node("ekf_node") {
    ekf_ = std::make_shared<EKF_SLAM>();

    // Parametri EKF
    this->declare_parameter("process_noise_v", 0.1);
    this->declare_parameter("process_noise_omega", 0.05);
    this->declare_parameter("meas_noise_range", 0.05);
    this->declare_parameter("meas_noise_bearing", 0.01);
    this->declare_parameter("association_threshold", 9.0); // Mahalanobis per loop closure

    ekf_->setProcessNoise(this->get_parameter("process_noise_v").as_double(),
                          this->get_parameter("process_noise_omega").as_double());
    ekf_->setMeasurementNoise(this->get_parameter("meas_noise_range").as_double(),
                              this->get_parameter("meas_noise_bearing").as_double());
    ekf_->setAssociationThreshold(this->get_parameter("association_threshold").as_double());

    RCLCPP_INFO(this->get_logger(), "EKF SLAM Node (Pacsim Edition) initialized");

    // --- SUBSCRIBERS (Adattati ai messaggi di Pacsim) ---
    // NOTA: Sostituisci "/pacsim/odometry" e "/pacsim/cones" con i nomi reali dei topic del simulatore
    sub_odom_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10, std::bind(&EKFNode::odomCallback, this, _1)); 

    sub_cones_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/pacsim/perception/livox_front/visualization", 10, std::bind(&EKFNode::conesCallback, this, _1)); 

    // --- PUBLISHERS (Identici al Graph SLAM) ---
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_estimate", 10);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cones_estimate", 10);

    // Timer a 10Hz come nel Graph SLAM
    timer_viz_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&EKFNode::publishSlamResults, this));

    last_update_time_ = this->get_clock()->now();
    first_odom_ = true;
    last_v_ = 0.0;
    last_omega_ = 0.0;
  }

private:
  // --- CALLBACK ODOMETRIA (Ora legge TwistWithCovarianceStamped) ---
  void odomCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    rclcpp::Time now(msg->header.stamp, this->get_clock()->get_clock_type());

    if (first_odom_) {
      last_update_time_ = now;
      first_odom_ = false;
      return;
    }

    double dt = (now - last_update_time_).seconds();
    
    // Estrae v e omega direttamente dal Twist
    last_v_ = msg->twist.twist.linear.x;
    last_omega_ = msg->twist.twist.angular.z;

    if (dt > 0.0) {
       ekf_->predict(last_v_, last_omega_, dt);
       last_update_time_ = now;
    }
  }

void conesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    
    if (first_odom_ || msg->markers.empty()) return; 

    rclcpp::Time msg_time(msg->markers[0].header.stamp, this->get_clock()->get_clock_type());
    double dt = (msg_time - last_update_time_).seconds();

    if (dt > 0.0) {
        ekf_->predict(last_v_, last_omega_, dt);
        last_update_time_ = msg_time;
    }

    int m = msg->markers.size();
    
    // CREIAMO SOLO 2 VETTORI (Niente più ids!)
    Eigen::VectorXd ranges(m);
    Eigen::VectorXd bearings(m);

    int valid_count = 0;
    for (int i = 0; i < m; i++) {
        double x = msg->markers[i].pose.position.x;
        double y = msg->markers[i].pose.position.y;
        
        double r = std::hypot(x, y);
        double b = std::atan2(y, x);

        if (std::isnan(r) || std::isnan(b)) continue;
        if (r > 15.0 || r < 0.1) continue; //da 20 a 15 per evitare di aggiungere coni troppo lontani

        ranges(valid_count)   = r;
        bearings(valid_count) = b;
        valid_count++;
    }
    
    if (valid_count > 0) {
        ranges.conservativeResize(valid_count);
        bearings.conservativeResize(valid_count);
        
        // CHIAMATA CORRETTA CON 2 ARGOMENTI
        ekf_->correct(ranges, bearings);
    }
  }

  // --- PUBBLICAZIONE RISULTATI (Pulita e con DELETEALL) ---
  void publishSlamResults() {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    
    // 1. Pubblica Posa
    auto s = ekf_->getState();
    geometry_msgs::msg::PoseStamped pst;
    pst.header.stamp = this->get_clock()->now();
    pst.header.frame_id = "map"; // Assumiamo che il simulatore pubblichi il frame 'map'
    pst.pose.position.x = s(0); 
    pst.pose.position.y = s(1); 
    pst.pose.position.z = 0.0;
    
    double cy = std::cos(s(2) * 0.5);
    double sy = std::sin(s(2) * 0.5);
    pst.pose.orientation.x = 0.0;
    pst.pose.orientation.y = 0.0;
    pst.pose.orientation.z = sy;
    pst.pose.orientation.w = cy;
    pub_pose_->publish(pst);

    // 2. Pubblica Coni Mappati
    visualization_msgs::msg::MarkerArray markers_msg;
    
    // --- TRUCCO GRAFICO (DELETEALL) ---
    // Cancella tutti i coni vecchi prima di disegnare quelli aggiornati
    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.header.frame_id = "map";
    delete_all_marker.header.stamp = pst.header.stamp;
    delete_all_marker.ns = "ekf_cones";
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers_msg.markers.push_back(delete_all_marker);

    int n = ekf_->numLandmarks();
    for (int i = 0; i < n; i++) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = pst.header.stamp;
      m.ns = "ekf_cones";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::CYLINDER; 
      m.action = visualization_msgs::msg::Marker::ADD;
      
      auto L = ekf_->getLandmarkPosition(i);
      m.pose.position.x = L(0); 
      m.pose.position.y = L(1); 
      m.pose.position.z = 0.0;
      m.pose.orientation.w = 1.0;
      
      m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.5;
      m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 1.0; // Coni Azzurri EKF
      m.lifetime = rclcpp::Duration::from_seconds(0);
      
      markers_msg.markers.push_back(m);
    }
    pub_markers_->publish(markers_msg);
  }

  // Interfacce semplificate
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_odom_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_cones_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  
  rclcpp::TimerBase::SharedPtr timer_viz_;

  std::shared_ptr<EKF_SLAM> ekf_;
  std::mutex ekf_mutex_;
  rclcpp::Time last_update_time_; 
  bool first_odom_;
  double last_v_;
  double last_omega_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}




