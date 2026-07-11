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
#include <deque>
#include <mutex>
#include <memory>

#include "ekf_slam/EKF_Pose.h"

using std::placeholders::_1;

class EKFPoseNode : public rclcpp::Node {
public:
  EKFPoseNode(const rclcpp::NodeOptions & options) : Node("ekf_pose_node", options) {
    ekf_ = std::make_shared<EKFPose>();

    this->declare_parameter("process_noise_v", 0.05);
    this->declare_parameter("process_noise_vy", 0.05);
    this->declare_parameter("process_noise_omega", 0.005);
    // Dichiara il parametro per il threshold
    this->declare_parameter("association_threshold", 3.9);

    double nv  = this->get_parameter("process_noise_v").as_double();
    double nvy = this->get_parameter("process_noise_vy").as_double();
    double nw  = this->get_parameter("process_noise_omega").as_double();
    double threshold = this->get_parameter("association_threshold").as_double();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ekf_->setProcessNoise(nv, nvy, nw);

    // sub e pub ai topic di interesse
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/pacsim/imu/cog_imu", 10, std::bind(&EKFPoseNode::imuCallback, this, _1));
    sub_cones_ = this->create_subscription<pacsim::msg::PerceptionDetections>("/pacsim/perception/livox_front/landmarks", 10, std::bind(&EKFPoseNode::conesCallback, this, _1));
    // Sottoscrizione alla mappa globale (Ground Truth)
    sub_map_ = this->create_subscription<pacsim::msg::Track>("/pacsim/track/landmarks", 10, std::bind(&EKFPoseNode::mapCallback, this, std::placeholders::_1));
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose_only", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odometry", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/trajectory", 10);

    timer_viz_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&EKFPoseNode::publishOdometry, this));

    first_odom_ = true;
    RCLCPP_INFO(this->get_logger(), "EKF Node pulito e avviato.");
  }

private:

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);

    rclcpp::Time now(msg->header.stamp, this->get_clock()->get_clock_type());
    if (first_odom_) { last_update_time_ = now; first_odom_ = false; return; }
    double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) return;

    // 1. Salva input IMU nello storico
    ImuRecord imu_rec;
    imu_rec.stamp = now;
    imu_rec.ax = msg->linear_acceleration.x;
    imu_rec.ay = msg->linear_acceleration.y;
    imu_rec.gyro_z = msg->angular_velocity.z;
    imu_rec.dt = dt;
    imu_buffer_.push_back(imu_rec);

    // 2. Esegui la predizione
    ekf_->predict(imu_rec.ax, imu_rec.ay, imu_rec.gyro_z, imu_rec.dt);
    
    // 3. Salva lo stato risultante nello storico
    StateRecord state_rec;
    state_rec.stamp = now;
    state_rec.state = ekf_->getState();
    state_rec.cov = ekf_->getCovariance();
    state_buffer_.push_back(state_rec);

    // 4. Mantieni i buffer leggeri (cancelliamo i dati più vecchi di 1 secondo)
    while(state_buffer_.size() > 0 && (now - state_buffer_.front().stamp).seconds() > 1.0) {
        state_buffer_.pop_front();
        imu_buffer_.pop_front();
    }

    last_update_time_ = now;
}

  void conesCallback(const pacsim::msg::PerceptionDetections::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    
    if (first_odom_ || state_buffer_.empty()) return; 

    rclcpp::Time lidar_time(msg->header.stamp, this->get_clock()->get_clock_type());
    
    // --- FASE 1: REWIND (Trova lo stato nel passato) ---
    auto best_state_it = state_buffer_.end();
    double min_time_diff = 1000.0;
    
    for (auto it = state_buffer_.begin(); it != state_buffer_.end(); ++it) {
        double diff = std::abs((it->stamp - lidar_time).seconds());
        if (diff < min_time_diff) {
            min_time_diff = diff;
            best_state_it = it;
        }
    }

    // Se non troviamo uno stato coerente (es. ritardo assurdo > 0.5s), ignoriamo la misura
    if (best_state_it == state_buffer_.end() || min_time_diff > 0.5) return;

    // Riavvolgiamo matematicamente l'EKF a quell'istante
    ekf_->setState(best_state_it->state);
    ekf_->setCovariance(best_state_it->cov);

    Eigen::VectorXd state = ekf_->getState();
    double pred_x = state(0);
    double pred_y = state(1);
    double pred_th = state(2);
    double threshold = this->get_parameter("association_threshold").as_double();

    // --- FASE 2: CORREZIONE STORICA ---
    for (const auto& perceived_cone : msg->detections) {
        double local_x = perceived_cone.pose.pose.position.x;
        double local_y = perceived_cone.pose.pose.position.y;

        double global_cone_x = pred_x + (local_x * std::cos(pred_th) - local_y * std::sin(pred_th));
        double global_cone_y = pred_y + (local_x * std::sin(pred_th) + local_y * std::cos(pred_th));

        double map_cone_x, map_cone_y;
        if (findNearestMapCone(global_cone_x, global_cone_y, threshold, map_cone_x, map_cone_y)) {
            
            // Addio Finto GPS, passiamo le coordinate crude!
            ekf_->correctPosition(map_cone_x, map_cone_y, local_x, local_y);
            
            state = ekf_->getState();
            pred_x = state(0);
            pred_y = state(1);
            pred_th = state(2);
        }
    }

    // Salviamo il nuovo stato corretto all'interno del buffer in quella posizione temporale
    best_state_it->state = ekf_->getState();
    best_state_it->cov = ekf_->getCovariance();

    // --- FASE 3: REPLAY (Propagazione in avanti) ---
    // Ritroviamo l'indice IMU corrispondente e avanziamo di 1 (perché quell'IMU è già stata usata)
    auto imu_it = imu_buffer_.begin() + std::distance(state_buffer_.begin(), best_state_it);
    if (imu_it != imu_buffer_.end()) ++imu_it; 

    auto state_update_it = best_state_it + 1;
    
    // Riapplichiamo iterativamente le letture IMU accumulate fino ad arrivare al presente
    while (imu_it != imu_buffer_.end() && state_update_it != state_buffer_.end()) {
        ekf_->predict(imu_it->ax, imu_it->ay, imu_it->gyro_z, imu_it->dt);
        
        // Aggiorniamo la storia con le nuove traiettorie modificate dalla correzione
        state_update_it->state = ekf_->getState();
        state_update_it->cov = ekf_->getCovariance();
        
        ++imu_it;
        ++state_update_it;
    }
}

  bool findNearestMapCone(double query_x, double query_y, double threshold, double& out_x, double& out_y) {
    double min_dist = threshold;
    bool found = false;

    // global_map_ rappresenta la tua lista di coni (es. std::vector<Point>) caricata in memoria
    for (const auto& map_cone : global_map_) {
        // Distanza euclidea tra il cono proiettato e il cono reale della mappa
        double dist = std::hypot(map_cone.x - query_x, map_cone.y - query_y);
        
        if (dist < min_dist) {
            min_dist = dist;
            out_x = map_cone.x;
            out_y = map_cone.y;
            found = true;
        }
    }
    return found;
}

  void mapCallback(const pacsim::msg::Track::SharedPtr msg) {
    // Se abbiamo già scaricato la mappa, ignoriamo i messaggi successivi
    if (map_received_) return; 

    global_map_.clear();

    // Creiamo una funzione lambda per estrarre i coni da un generico array
    auto extractCones = [&](const std::vector<pacsim::msg::Landmark>& cone_array) {
        for (const auto& cone : cone_array) {
            Point2D p;
            // Navighiamo la struttura PoseWithCovariance
            p.x = cone.pose.pose.position.x; 
            p.y = cone.pose.pose.position.y;
            global_map_.push_back(p);
        }
    };

    // Estraiamo i coni da tutte le liste pertinenti della pista
    extractCones(msg->left_lane);
    extractCones(msg->right_lane);
    extractCones(msg->unknown);
    
    // Opzionale: se ti servono anche i gate di cronometraggio per la localizzazione
    // extractCones(msg->time_keeping_gates);

    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Mappa globale acquisita con successo! Totale coni: %zu", global_map_.size());
}

  void publishOdometry() {
    if (first_odom_) return;
    
    Eigen::VectorXd state;
    Eigen::MatrixXd P;

    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        if (first_odom_) return;
        state = ekf_->getState();
        P = ekf_->getCovariance();
    }

    RCLCPP_INFO(this->get_logger(), "Bias stimato (rad/s): %f", state(5));
    
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "ekf_estimate";

    // Mappa stato in Pose
    odom.pose.pose.position.x = state(0);
    odom.pose.pose.position.y = state(1);
    odom.pose.pose.orientation.z = std::sin(state(2) * 0.5);
    odom.pose.pose.orientation.w = std::cos(state(2) * 0.5);

    // Mappa Covarianza P (6x6) in Pose Covariance (36 elementi)
    // L'ordine ROS è: [x, y, z, roll, pitch, yaw]
    // La tua matrice P è: [x, y, theta, vx, vy, omega]
    // Dobbiamo mappare le righe/colonne 0,1,2 di P negli indici corrispondenti
    
    // Inizializza covarianza a 0
    odom.pose.covariance.fill(0.0);
    
    // Mappa x, y, yaw (che è l'indice 2 del tuo stato)
    // ROS Odometry richiede x=0, y=1, yaw=5 (nelle ultime 3 posizioni di 6)
    odom.pose.covariance[0]  = P(0,0); // Cov(x,x)
    odom.pose.covariance[1]  = P(0,1); // Cov(x,y)
    odom.pose.covariance[5]  = P(0,2); // Cov(x,yaw)
    
    odom.pose.covariance[6]  = P(1,0); // Cov(y,x)
    odom.pose.covariance[7]  = P(1,1); // Cov(y,y)
    odom.pose.covariance[11] = P(1,2); // Cov(y,yaw)
    
    odom.pose.covariance[30] = P(2,0); // Cov(yaw,x)
    odom.pose.covariance[31] = P(2,1); // Cov(yaw,y)
    odom.pose.covariance[35] = P(2,2); // Cov(yaw,yaw)

    pub_odom_->publish(odom);

    // --- AGGIUNGI QUESTO PER IL SDR IN MOVIMENTO ---
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = odom.header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "ekf_estimate"; // Il nome del tuo SDR

    t.transform.translation.x = state(0);
    t.transform.translation.y = state(1);
    t.transform.translation.z = 0.0;
    
    // Usiamo la stessa orientazione che abbiamo già calcolato
    t.transform.rotation = odom.pose.pose.orientation; 

    tf_broadcaster_->sendTransform(t);

    // Configura l'header del Path
    path_msg_.header.stamp = this->now();
    path_msg_.header.frame_id = "map";

    // Crea un nuovo punto (PoseStamped) estraendo i dati da Odom
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header = path_msg_.header;
    current_pose.pose = odom.pose.pose;

    // Aggiungi il punto alla lista
    path_msg_.poses.push_back(current_pose);

    // Pubblica l'intera traiettoria
    pub_path_->publish(path_msg_);
  }

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

  std::vector<Point2D> global_map_;
  bool map_received_ = false; // Flag per non ricalcolare la mappa a ogni ciclo
  std::deque<StateRecord> state_buffer_;
  std::deque<ImuRecord> imu_buffer_;
    
  rclcpp::Subscription<pacsim::msg::Track>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<pacsim::msg::PerceptionDetections>::SharedPtr sub_cones_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  // ====== gestione disegno traccia ======
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  nav_msgs::msg::Path path_msg_;
  // ======================================
  rclcpp::TimerBase::SharedPtr timer_viz_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<EKFPose> ekf_;
  std::mutex ekf_mutex_;
  rclcpp::Time last_update_time_;
  bool first_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides({{"use_sim_time", true}});
  rclcpp::spin(std::make_shared<EKFPoseNode>(options));
  rclcpp::shutdown();
  return 0;
}