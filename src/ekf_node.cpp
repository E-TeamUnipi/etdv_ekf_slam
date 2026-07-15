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

    this->declare_parameter("process_noise_v", 0.05);
    this->declare_parameter("process_noise_vy", 0.05);
    this->declare_parameter("process_noise_omega", 0.005);
    this->declare_parameter("lidar_noise_x", 0.03);
    this->declare_parameter("lidar_noise_y", 0.03);
    // Dichiara il parametro per il threshold
    this->declare_parameter("association_threshold", 3.9);
    this->declare_parameter("imu_yaw_offset", 0.0);
    this->declare_parameter("max_distance", 15.0);

    double nv  = this->get_parameter("process_noise_v").as_double();
    double nvy = this->get_parameter("process_noise_vy").as_double();
    double nw  = this->get_parameter("process_noise_omega").as_double();
    double nlx  = this->get_parameter("lidar_noise_x").as_double();
    double nly = this->get_parameter("lidar_noise_y").as_double();
    threshold = this->get_parameter("association_threshold").as_double();
    imu_yaw_offset = this->get_parameter("imu_yaw_offset").as_double();
    max_distance = this->get_parameter("max_distance").as_double();

    // --- LOG DEI PARAMETRI (Sanity Check) ---
    RCLCPP_INFO(this->get_logger(), 
        "\n======================================================\n"
        "🚀 NODO EKF SLAM AVVIATO - CHECK PARAMETRI ATTIVI\n"
        "======================================================\n"
        "[Rumore di Processo (Q)]\n"
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
        nv, nvy, nw, 
        nlx, nly, 
        threshold, max_distance,
        imu_yaw_offset
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ekf_->setProcessNoise(nv, nvy, nw, nlx, nly);

    // sub e pub ai topic di interesse
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/pacsim/imu/cog_imu", 10, std::bind(&EKFPoseNode::imuCallback, this, _1));
    sub_cones_ = this->create_subscription<pacsim::msg::PerceptionDetections>("/pacsim/perception/livox_front/landmarks", 10, std::bind(&EKFPoseNode::conesCallback, this, _1));
    // Sottoscrizione alla mappa globale (Ground Truth)
    sub_map_ = this->create_subscription<pacsim::msg::Track>("/pacsim/track/landmarks", 10, std::bind(&EKFPoseNode::mapCallback, this, std::placeholders::_1));
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose_only", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odometry", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/trajectory", 10);
    pub_map_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ekf/map_cones", 10);
    pub_map_rmse_ = this->create_publisher<std_msgs::msg::Float64>("/ekf/map_rmse", 10);
    pub_latency_ = this->create_publisher<std_msgs::msg::Float64>("/ekf/latency_ms", 10);
    pub_imu_latency_ = this->create_publisher<std_msgs::msg::Float64>("/ekf/imu_latency_ms", 10);

    // timer_viz_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&EKFPoseNode::publishOdometry, this));
    timer_map_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&EKFPoseNode::publishMap, this));

    first_odom_ = true;
    RCLCPP_INFO(this->get_logger(), "EKF Node pulito e avviato.");
  }

private:

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    auto start_time = std::chrono::high_resolution_clock::now();

    std::lock_guard<std::mutex> lock(ekf_mutex_);

    rclcpp::Time now(msg->header.stamp, this->get_clock()->get_clock_type());
    if (first_odom_) { last_update_time_ = now; first_odom_ = false; return; }
    double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) return;

    // 1. Salva input IMU nello storico
    ImuRecord imu_rec;
    imu_rec.stamp = now;

    double ax_raw = msg->linear_acceleration.x;
    double ay_raw = msg->linear_acceleration.y;
    
    // Rotazione per compensare il montaggio fisico
    imu_rec.ax = ax_raw * std::cos(imu_yaw_offset) - ay_raw * std::sin(imu_yaw_offset);
    imu_rec.ay = ax_raw * std::sin(imu_yaw_offset) + ay_raw * std::cos(imu_yaw_offset);
    
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

    // 4. Mantieni i buffer leggeri in modo indipendente (finestra di 1 secondo)
    while (!state_buffer_.empty() && (now - state_buffer_.front().stamp).seconds() > 1.0) {
        state_buffer_.pop_front();
    }
    while (!imu_buffer_.empty() && (now - imu_buffer_.front().stamp).seconds() > 1.0) {
        imu_buffer_.pop_front();
    }

    last_update_time_ = now;
    publishOdometry(now);

    // 2. Ferma il cronometro e pubblica
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    
    std_msgs::msg::Float64 lat_msg;
    lat_msg.data = duration_us / 1000.0;
    pub_imu_latency_->publish(lat_msg);

    accumulated_imu_latency_ms_ += lat_msg.data;
    imu_latency_counter_++;

    // Stampa la media ogni 100 callback (essendo l'IMU ad alta frequenza)
    if (imu_latency_counter_ >= 100) {
        double avg_latency = accumulated_imu_latency_ms_ / 100.0;
        // RCLCPP_INFO(this->get_logger(), "⚡ IMU Performance: Latenza media = %.4f ms", avg_latency);
        imu_latency_counter_ = 0;
        accumulated_imu_latency_ms_ = 0.0;
    }
}

void conesCallback(const pacsim::msg::PerceptionDetections::SharedPtr msg) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    
    if (first_odom_ || state_buffer_.empty()) return; 

    // Il timestamp del messaggio punta GIA' al passato corretto (es. T - 200ms)
    rclcpp::Time lidar_time(msg->header.stamp, this->get_clock()->get_clock_type());
    
    // --- FASE 1: REWIND (Cerca lo stato al tempo del lidar_time) ---
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

    // Riavvolgiamo matematicamente l'EKF a quell'istante
    ekf_->setState(best_state_it->state);
    ekf_->setCovariance(best_state_it->cov);

    // --- FASE 2: CORREZIONE STORICA ---
    for (const auto& perceived_cone : msg->detections) {
        double local_x = perceived_cone.pose.pose.position.x;
        double local_y = perceived_cone.pose.pose.position.y;

        // --- NUOVO: FILTRO SPAZIALE (Cut-off a 15 metri) ---
        // std::hypot calcola la distanza euclidea dall'origine del sensore
        double distance = std::hypot(local_x, local_y);
        if (distance > max_distance) {
            continue; // Il cono è troppo lontano, ignoralo e passa al prossimo
        }

        int matched_id = ekf_->dataAssociation(local_x, local_y, threshold);

        if (matched_id >= 0) {
            ekf_->correctPosition(matched_id, local_x, local_y);
        } else {
            ekf_->addNewLandmark(local_x, local_y);
        }
    }

    best_state_it->state = ekf_->getState();
    best_state_it->cov = ekf_->getCovariance();

    // --- FASE 3: REPLAY (Propagazione in avanti Timestamp-Driven) ---
    rclcpp::Time replay_start_time = best_state_it->stamp;
    auto state_it = best_state_it;

    for (auto& imu_rec : imu_buffer_) {
        // Ignoriamo le letture IMU avvenute prima o esattamente allo stato appena corretto
        if (imu_rec.stamp <= replay_start_time) {
            continue; 
        }

        // Reintegriamo la fisica dal passato verso il presente
        ekf_->predict(imu_rec.ax, imu_rec.ay, imu_rec.gyro_z, imu_rec.dt);
        
        // Sincronizzazione Lineare O(N): avanziamo l'iteratore dello stato senza ricominciare da capo
        while (state_it != state_buffer_.end() && state_it->stamp < imu_rec.stamp) {
            ++state_it;
        }
        if (state_it != state_buffer_.end() && state_it->stamp == imu_rec.stamp) {
            state_it->state = ekf_->getState();
            state_it->cov = ekf_->getCovariance();
        }
    }

    // --- Calcolo Performance (invariato) ---
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    
    std_msgs::msg::Float64 lat_msg;
    lat_msg.data = duration_us / 1000.0;
    pub_latency_->publish(lat_msg);

    accumulated_latency_ms_ += (duration_us / 1000.0);
    latency_counter_++;

    if (latency_counter_ >= 10) {
        double avg_latency = accumulated_latency_ms_ / 10.0;
        // Commentato/abilitato in base alle tue preferenze
        // RCLCPP_INFO(this->get_logger(), "⚡ Performance: Latenza media = %.3f ms", avg_latency);
        latency_counter_ = 0;
        accumulated_latency_ms_ = 0.0;
    }
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

void publishOdometry(rclcpp::Time stamp) {
    if (first_odom_) return;
    
    Eigen::VectorXd state;
    Eigen::MatrixXd P;

    state = ekf_->getState();
    P = ekf_->getCovariance();

    // RCLCPP_INFO(this->get_logger(), "Bias stimato (rad/s): %f", state(5));9

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "cog";

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
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "ekf_cog"; // Il nome del tuo SDR

    t.transform.translation.x = state(0);
    t.transform.translation.y = state(1);
    t.transform.translation.z = 0.0;
    
    // Usiamo la stessa orientazione che abbiamo già calcolato
    t.transform.rotation = odom.pose.pose.orientation; 

    tf_broadcaster_->sendTransform(t);

    // Configura l'header del Path
    path_msg_.header.stamp = stamp;
    path_msg_.header.frame_id = "map";

    // Crea un nuovo punto (PoseStamped) estraendo i dati da Odom
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header = path_msg_.header;
    current_pose.pose = odom.pose.pose;

    // Aggiungi il punto alla lista
    path_msg_.poses.push_back(current_pose);

    // LIMITA LA LUNGHEZZA DEL PATH (es. massimo 1000 punti)
    // if (path_msg_.poses.size() > 1000) {
    //     path_msg_.poses.erase(path_msg_.poses.begin());
    // }

    // Pubblica l'intera traiettoria
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
    // Se ci sono solo i 6 stati della vettura, non abbiamo ancora mappato nulla
    if (state_size <= 6) return;

    int num_cones = (state_size - 6) / 2;
    visualization_msgs::msg::MarkerArray marker_array;

    for (int i = 0; i < num_cones; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.ns = "ekf_landmarks"; // Namespace del marker
        marker.id = i;               // ID univoco basato sulla posizione nel vettore
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Estraiamo le coordinate dal vettore di stato
        marker.pose.position.x = state(6 + 2 * i);
        marker.pose.position.y = state(6 + 2 * i + 1);
        marker.pose.position.z = 0.0; // Poggiano a terra
        
        // Orientazione neutra
        marker.pose.orientation.w = 1.0;

        // Dimensioni del cilindro (diametro 20cm, altezza 30cm)
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.3;
        
        // Colore RGBA: Verde brillante, completamente opaco
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        
        marker_array.markers.push_back(marker);
    }
    
    pub_map_->publish(marker_array);
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
  // ====== gestione disegno traccia =======
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  // ====== errore mappa =======
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_map_rmse_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_latency_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_imu_latency_;
  // ===========================
  nav_msgs::msg::Path path_msg_;
  // =======================================
  // ====== gestione disegno landmark ======
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_map_;
  // =======================================
  rclcpp::TimerBase::SharedPtr timer_viz_;
  rclcpp::TimerBase::SharedPtr timer_map_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<EKFPose> ekf_;
  std::mutex ekf_mutex_;
  rclcpp::Time last_update_time_;
  bool first_odom_;
  double threshold;
  double imu_yaw_offset;
  double max_distance;

  // calcolo performance
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