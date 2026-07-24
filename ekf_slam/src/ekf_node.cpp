#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "pacsim/msg/perception_detections.hpp"
#include "pacsim/msg/track.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <deque>
#include <mutex>
#include <memory>

#include "ekf_slam/EKF_Pose.h"

using std::placeholders::_1;

class EKFPoseNode : public rclcpp::Node {
public:
  EKFPoseNode(const rclcpp::NodeOptions & options) : Node("ekf_pose_node", options) {
    ekf_ = std::make_shared<EKFPose>();

    // --- NUOVI PARAMETRI PER LA MODALITA' E I TOPIC ---
    this->declare_parameter("use_sim_perception", true);
    this->declare_parameter<std::string>("perception_topic_sim", "/pacsim/perception/livox_front/landmarks");
    this->declare_parameter<std::string>("perception_topic_real", "/perception/cones");

    // --- PARAMETRI DI TUNING ORIGINALI ---
    this->declare_parameter("process_noise_x", 0.0001);
    this->declare_parameter("process_noise_y", 0.0001);
    this->declare_parameter("process_noise_yaw", 0.0001);
    this->declare_parameter("process_noise_v", 0.05);
    this->declare_parameter("process_noise_vy", 0.05);
    this->declare_parameter("process_noise_omega", 0.005);
    this->declare_parameter("lidar_noise_x", 0.03);
    this->declare_parameter("lidar_noise_y", 0.03);
    this->declare_parameter("association_threshold", 3.9);
    this->declare_parameter("imu_yaw_offset", 0.0);
    this->declare_parameter("max_distance", 15.0);

    bool use_sim_perception = this->get_parameter("use_sim_perception").as_bool();
    std::string perception_topic_sim = this->get_parameter("perception_topic_sim").as_string();
    std::string perception_topic_real = this->get_parameter("perception_topic_real").as_string();

    double x  = this->get_parameter("process_noise_x").as_double();
    double y = this->get_parameter("process_noise_y").as_double();
    double yaw  = this->get_parameter("process_noise_yaw").as_double();
    double nv  = this->get_parameter("process_noise_v").as_double();
    double nvy = this->get_parameter("process_noise_vy").as_double();
    double nw  = this->get_parameter("process_noise_omega").as_double();
    double nlx  = this->get_parameter("lidar_noise_x").as_double();
    double nly = this->get_parameter("lidar_noise_y").as_double();
    threshold = this->get_parameter("association_threshold").as_double();
    imu_yaw_offset = this->get_parameter("imu_yaw_offset").as_double();
    max_distance = this->get_parameter("max_distance").as_double();

    // --- LOG DEI PARAMETRI ---
    RCLCPP_INFO(this->get_logger(), 
        "\n======================================================\n"
        "🚀 NODO EKF SLAM AVVIATO - CHECK PARAMETRI ATTIVI\n"
        "======================================================\n"
        "[Modalità Operativa]\n"
        "  - Esecuzione        : %s\n"
        "[Rumore di Processo (Q)]\n"
        "  - x (Longitudinale) : %.4f\n"
        "  - y (Laterale)      : %.4f\n"
        "  - yaw (Heading)     : %.4f\n"
        "  - v (Longitudinale) : %.4f\n"
        "  - vy (Laterale)     : %.4f\n"
        "  - omega (Rotazione) : %.4f\n"
        "[Rumore di Misura (R)]\n"
        "  - LiDAR x           : %.4f\n"
        "  - LiDAR y           : %.4f\n"
        "[Data Association & Filtri]\n"
        "  - Soglia Mahalanobis: %.2f\n"
        "  - Max Lidar Range   : %.2f m\n"
        "[Geometria & Calibrazione (Offset)]\n"
        "  - LiDAR Yaw Offset  : %.4f rad\n"
        "======================================================",
        use_sim_perception ? "SIMULATORE (PacSim)" : "REALE (MarkerArray)",
        x, y, yaw, nv, nvy, nw, nlx, nly, threshold, max_distance, imu_yaw_offset
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ekf_->setProcessNoise(x, y, yaw, nv, nvy, nw, nlx, nly);

    // sub e pub ai topic di interesse
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/pacsim/imu/cog_imu", 10, std::bind(&EKFPoseNode::imuCallback, this, _1));
    
    // --- SUBSCRIBER CONDIZIONALE DELLA PERCEZIONE ---
    if (use_sim_perception) {
        sub_cones_sim_ = this->create_subscription<pacsim::msg::PerceptionDetections>(
            perception_topic_sim, 10, std::bind(&EKFPoseNode::conesCallbackSim, this, _1)
        );
    } else {
        sub_cones_real_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            perception_topic_real, 10, std::bind(&EKFPoseNode::conesCallbackReal, this, _1)
        );
    }

    // Sottoscrizione alla mappa globale (Ground Truth)
    sub_map_ = this->create_subscription<pacsim::msg::Track>("/pacsim/track/landmarks", 10, std::bind(&EKFPoseNode::mapCallback, this, std::placeholders::_1));
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose_only", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odometry", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/trajectory", 10);
    pub_map_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ekf/map_cones", 10);
    pub_map_rmse_ = this->create_publisher<std_msgs::msg::Float64>("/ekf/map_rmse", 10);
    pub_latency_ = this->create_publisher<std_msgs::msg::Float64>("/ekf/latency_ms", 10);
    pub_imu_latency_ = this->create_publisher<std_msgs::msg::Float64>("/ekf/imu_latency_ms", 10);

    // 10 Hz - Lento, per non ricaricare la mappa continuamente
    timer_map_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() { this->publishMap(); });
    
    // 50 Hz - Veloce, fluido e costante per il 3D di Foxglove (sganciato dai microscatti dell'IMU)
    timer_odom_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.02), [this]() { this->publishOdometry(); });
    
    // 20 Hz - Medio, sufficiente per vedere la scia del Path senza distruggere la rete
    timer_path_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.05), [this]() { this->publishPath(); });

    first_odom_ = true;
    RCLCPP_INFO(this->get_logger(), "EKF Node pulito e avviato.");
  }

private:

  // =====================================================================
  // 1. DEFINIZIONE DELLE STRUTTURE (Devono stare in alto per lo scope!)
  // =====================================================================
  struct Point2D {
      double x;
      double y;
  };
  
  struct ImuRecord {
      rclcpp::Time stamp;
      double ax;
      double ay;
      double gyro_z;
      double dt;
  };

  struct StateRecord {
      rclcpp::Time stamp;
      Eigen::VectorXd state;
      Eigen::MatrixXd cov;
  };


  // =====================================================================
  // 2. CALLBACK: IMU
  // =====================================================================
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
      auto start_time = std::chrono::high_resolution_clock::now();
      std::lock_guard<std::mutex> lock(ekf_mutex_);

      rclcpp::Time now(msg->header.stamp, this->get_clock()->get_clock_type());
      if (first_odom_) { last_update_time_ = now; first_odom_ = false; return; }
      double dt = (now - last_update_time_).seconds();
      if (dt <= 0.0) return;

      ImuRecord imu_rec;
      imu_rec.stamp = now;

      double ax_raw = msg->linear_acceleration.x;
      double ay_raw = msg->linear_acceleration.y;
      
      imu_rec.ax = ax_raw * std::cos(imu_yaw_offset) - ay_raw * std::sin(imu_yaw_offset);
      imu_rec.ay = ax_raw * std::sin(imu_yaw_offset) + ay_raw * std::cos(imu_yaw_offset);
      
      imu_rec.gyro_z = msg->angular_velocity.z;
      imu_rec.dt = dt;
      imu_buffer_.push_back(imu_rec);

      ekf_->predict(imu_rec.ax, imu_rec.ay, imu_rec.gyro_z, imu_rec.dt);
      
      StateRecord state_rec;
      state_rec.stamp = now;
      state_rec.state = ekf_->getState();
      state_rec.cov = ekf_->getCovariance();
      state_buffer_.push_back(state_rec);

      while (!state_buffer_.empty() && (now - state_buffer_.front().stamp).seconds() > 1.0) {
          state_buffer_.pop_front();
      }
      while (!imu_buffer_.empty() && (now - imu_buffer_.front().stamp).seconds() > 1.0) {
          imu_buffer_.pop_front();
      }

      last_update_time_ = now;
    //   publishOdometry(now);

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
      
      std_msgs::msg::Float64 lat_msg;
      lat_msg.data = duration_us / 1000.0;
      pub_imu_latency_->publish(lat_msg);

      accumulated_imu_latency_ms_ += lat_msg.data;
      imu_latency_counter_++;

      if (imu_latency_counter_ >= 100) {
          imu_latency_counter_ = 0;
          accumulated_imu_latency_ms_ = 0.0;
      }
  }


  // =====================================================================
  // 3. LOGICA CORE UNIFICATA EKF (Rewind, Update, Replay)
  // (Tipo di start_time corretto in std::chrono invece di auto)
  // =====================================================================
  void processConesCore(const std::vector<Point2D>& perceived_cones, rclcpp::Time lidar_time, std::chrono::time_point<std::chrono::high_resolution_clock> start_time) {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      
      if (first_odom_ || state_buffer_.empty()) return; 

      // --- FASE 1: REWIND ---
      auto best_state_it = state_buffer_.end();
      double min_time_diff = 1000.0;
      
      for (auto it = state_buffer_.begin(); it != state_buffer_.end(); ++it) {
          double diff = std::abs((it->stamp - lidar_time).seconds());
          if (diff < min_time_diff) {
              min_time_diff = diff;
              best_state_it = it;
          }
      }

      if (best_state_it == state_buffer_.end() || min_time_diff > 0.5) return;

      ekf_->setState(best_state_it->state);
      ekf_->setCovariance(best_state_it->cov);

      // --- FASE 2: CORREZIONE STORICA ---
      for (const auto& cone : perceived_cones) {
          double distance = std::hypot(cone.x, cone.y);
          if (distance > max_distance) {
              continue; 
          }

          int matched_id = ekf_->dataAssociation(cone.x, cone.y, threshold);

          if (matched_id >= 0) {
              ekf_->correctPosition(matched_id, cone.x, cone.y);
          } else {
              ekf_->addNewLandmark(cone.x, cone.y);
          }
      }

      best_state_it->state = ekf_->getState();
      best_state_it->cov = ekf_->getCovariance();

      // --- FASE 3: REPLAY ---
      rclcpp::Time replay_start_time = best_state_it->stamp;
      auto state_it = best_state_it;

      for (auto& imu_rec : imu_buffer_) {
          if (imu_rec.stamp <= replay_start_time) {
              continue; 
          }

          ekf_->predict(imu_rec.ax, imu_rec.ay, imu_rec.gyro_z, imu_rec.dt);
          
          while (state_it != state_buffer_.end() && state_it->stamp < imu_rec.stamp) {
              ++state_it;
          }
          if (state_it != state_buffer_.end() && state_it->stamp == imu_rec.stamp) {
              state_it->state = ekf_->getState();
              state_it->cov = ekf_->getCovariance();
          }
      }

      // --- Calcolo Performance ---
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
      
      std_msgs::msg::Float64 lat_msg;
      lat_msg.data = duration_us / 1000.0;
      pub_latency_->publish(lat_msg);

      accumulated_latency_ms_ += (duration_us / 1000.0);
      latency_counter_++;

      if (latency_counter_ >= 10) {
          latency_counter_ = 0;
          accumulated_latency_ms_ = 0.0;
      }
  }


  // =====================================================================
  // 4. PARSER: SIMULATORE (PacSim)
  // =====================================================================
  void conesCallbackSim(const pacsim::msg::PerceptionDetections::SharedPtr msg) {
      auto start_time = std::chrono::high_resolution_clock::now();
      
      std::vector<Point2D> extracted_cones;
      for (const auto& perceived_cone : msg->detections) {
          extracted_cones.push_back({perceived_cone.pose.pose.position.x, perceived_cone.pose.pose.position.y});
      }

      rclcpp::Time lidar_time(msg->header.stamp, this->get_clock()->get_clock_type());
      processConesCore(extracted_cones, lidar_time, start_time);
  }


  // =====================================================================
  // 5. PARSER: PERCEZIONE REALE (Rosbag / Vettura)
  // =====================================================================
  void conesCallbackReal(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      auto start_time = std::chrono::high_resolution_clock::now();
      
      if (msg->markers.empty()) return;

      std::vector<Point2D> extracted_cones;
      for (const auto& marker : msg->markers) {
          if (marker.action == visualization_msgs::msg::Marker::DELETEALL) continue;
          extracted_cones.push_back({marker.pose.position.x, marker.pose.position.y});
      }

      rclcpp::Time lidar_time(msg->markers[0].header.stamp, this->get_clock()->get_clock_type());
      processConesCore(extracted_cones, lidar_time, start_time);
  }


  // =====================================================================
  // 6. MAPPA E PUBLISHERS (Originali)
  // =====================================================================
  void mapCallback(const pacsim::msg::Track::SharedPtr msg) {
      if (map_received_) return; 
      global_map_.clear();

      auto extractCones = [&](const std::vector<pacsim::msg::Landmark>& cone_array) {
          for (const auto& cone : cone_array) {
              Point2D p;
              p.x = cone.pose.pose.position.x; 
              p.y = cone.pose.pose.position.y;
              global_map_.push_back(p);
          }
      };

      extractCones(msg->left_lane);
      extractCones(msg->right_lane);
      extractCones(msg->unknown);

      map_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Mappa globale acquisita con successo! Totale coni: %zu", global_map_.size());
  }

  void publishOdometry() {
      Eigen::VectorXd state;
      Eigen::MatrixXd P;
      rclcpp::Time stamp;
      
      // Estrazione sicura: blocchiamo l'EKF giusto un microsecondo per copiare i dati
      {
          std::lock_guard<std::mutex> lock(ekf_mutex_);
          if (first_odom_) return;
          state = ekf_->getState();
          P = ekf_->getCovariance();
          // Usiamo il tempo dell'ultimo aggiornamento IMU per una coerenza temporale perfetta della TF
          stamp = last_update_time_; 
      }

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "map";
      odom.child_frame_id = "cog";

      odom.pose.pose.position.x = state(0);
      odom.pose.pose.position.y = state(1);
      odom.pose.pose.orientation.z = std::sin(state(2) * 0.5);
      odom.pose.pose.orientation.w = std::cos(state(2) * 0.5);

      odom.pose.covariance.fill(0.0);
      
      odom.pose.covariance[0]  = P(0,0);
      odom.pose.covariance[1]  = P(0,1);
      odom.pose.covariance[5]  = P(0,2);
      
      odom.pose.covariance[6]  = P(1,0);
      odom.pose.covariance[7]  = P(1,1);
      odom.pose.covariance[11] = P(1,2);
      
      odom.pose.covariance[30] = P(2,0);
      odom.pose.covariance[31] = P(2,1);
      odom.pose.covariance[35] = P(2,2);

      pub_odom_->publish(odom);

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = "map";
      t.child_frame_id = "ekf_cog";

      t.transform.translation.x = state(0);
      t.transform.translation.y = state(1);
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom.pose.pose.orientation; 

      tf_broadcaster_->sendTransform(t);
  }

  void publishPath() {
      Eigen::VectorXd state;
      rclcpp::Time stamp = this->now();
      
      {
          std::lock_guard<std::mutex> lock(ekf_mutex_);
          if (first_odom_) return;
          state = ekf_->getState();
      }

      // Estraiamo la posa corrente dallo stato
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose.header.stamp = stamp;
      current_pose.header.frame_id = "map";
      current_pose.pose.position.x = state(0);
      current_pose.pose.position.y = state(1);
      current_pose.pose.orientation.z = std::sin(state(2) * 0.5);
      current_pose.pose.orientation.w = std::cos(state(2) * 0.5);

      path_msg_.header.stamp = stamp;
      path_msg_.header.frame_id = "map";
      path_msg_.poses.push_back(current_pose);

      // Manteniamo il limite rigido per non saturare la rete
      if (path_msg_.poses.size() > 1000) {
          path_msg_.poses.erase(path_msg_.poses.begin());
      }

      pub_path_->publish(path_msg_);
  }

  void publishMap() {
      Eigen::VectorXd state;
      
      {
          std::lock_guard<std::mutex> lock(ekf_mutex_);
          if (first_odom_) return;
          state = ekf_->getState();
      }

      int state_size = state.size();
      if (state_size <= 6) return;

      int num_cones = (state_size - 6) / 2;
      visualization_msgs::msg::MarkerArray marker_array;

      for (int i = 0; i < num_cones; ++i) {
          visualization_msgs::msg::Marker marker;
          marker.header.stamp = this->now();
          marker.header.frame_id = "map";
          marker.ns = "ekf_landmarks";
          marker.id = i;               
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          marker.action = visualization_msgs::msg::Marker::ADD;
          
          marker.pose.position.x = state(6 + 2 * i);
          marker.pose.position.y = state(6 + 2 * i + 1);
          marker.pose.position.z = 0.0; 
          
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
          marker.scale.z = 0.3;
          
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0f;
          
          marker_array.markers.push_back(marker);
      }
      
      pub_map_->publish(marker_array);
  }

  // =====================================================================
  // 7. VARIABILI DI CLASSE
  // =====================================================================
  std::vector<Point2D> global_map_;
  bool map_received_ = false; 
  std::deque<StateRecord> state_buffer_;
  std::deque<ImuRecord> imu_buffer_;
    
  rclcpp::Subscription<pacsim::msg::Track>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  
  // --- I DUE SUBSCRIBER CONDIZIONALI PER LA PERCEZIONE ---
  rclcpp::Subscription<pacsim::msg::PerceptionDetections>::SharedPtr sub_cones_sim_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_cones_real_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_map_rmse_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_latency_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_imu_latency_;
  
  nav_msgs::msg::Path path_msg_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_map_;
  
  rclcpp::TimerBase::SharedPtr timer_viz_;
  rclcpp::TimerBase::SharedPtr timer_map_;
  rclcpp::TimerBase::SharedPtr timer_odom_;
  rclcpp::TimerBase::SharedPtr timer_path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<EKFPose> ekf_;
  std::mutex ekf_mutex_;
  rclcpp::Time last_update_time_;
  bool first_odom_;
  double threshold;
  double imu_yaw_offset;
  double max_distance;

  int latency_counter_ = 0;
  double accumulated_latency_ms_ = 0.0;
  int imu_latency_counter_ = 0;
  double accumulated_imu_latency_ms_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides({{"use_sim_time", true}});
  rclcpp::spin(std::make_shared<EKFPoseNode>(options));
  rclcpp::shutdown();
  return 0;
}