#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mutex>
#include <memory>
#include <vector>
#include <cmath>

#include "ekf_slam/EKF_SLAM.h"

using std::placeholders::_1;

// Soglia di staleness odometria: se l'ultimo messaggio odom è più vecchio di
// questo valore, predict usa v=0 omega=0 per non propagare velocità fantasma.
static constexpr double ODOM_STALE_THRESHOLD_S = 1.5; //alzato da 0.5 per evitare gli stop di pacsim alla loop closure

class EKFNode : public rclcpp::Node {
public:
  EKFNode(const rclcpp::NodeOptions & options) : Node("ekf_node", options) {
    ekf_ = std::make_shared<EKF_SLAM>();

    this->declare_parameter("process_noise_v",      0.1);
    this->declare_parameter("process_noise_omega",  0.05);
    this->declare_parameter("meas_noise_range",     0.15);
    this->declare_parameter("meas_noise_bearing",   0.05);
    this->declare_parameter("association_threshold", 9.0);

    ekf_->setProcessNoise(
        this->get_parameter("process_noise_v").as_double(),
        this->get_parameter("process_noise_omega").as_double());
    ekf_->setMeasurementNoise(
        this->get_parameter("meas_noise_range").as_double(),
        this->get_parameter("meas_noise_bearing").as_double());
    ekf_->setAssociationThreshold(
        this->get_parameter("association_threshold").as_double());

    RCLCPP_INFO(this->get_logger(), "EKF SLAM Node (Pacsim Edition) initialized");

    // --- SUBSCRIBERS ---
    // FIX 6: queue size 1 per i coni — ogni messaggio è uno snapshot completo,
    // tenere messaggi vecchi in coda introduce solo ritardo.
    sub_odom_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10,
        std::bind(&EKFNode::odomCallback, this, _1));

    sub_cones_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/pacsim/perception/livox_front/visualization", 1,
        std::bind(&EKFNode::conesCallback, this, _1));

    // --- PUBLISHERS ---
    pub_pose_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_estimate", 10);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cones_estimate", 10);

    // FIX 1: rclcpp::create_timer (funzione libera) accetta il clock esplicito
    // ed è compatibile con tutte le distro ROS2 (Humble, Iron, Jazzy).
    // Con use_sim_time=true, this->get_clock() è il sim clock → il timer
    // segue il tempo simulato invece del wall clock.
    timer_viz_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration::from_seconds(0.1),
        std::bind(&EKFNode::publishSlamResults, this));

    last_update_time_ = this->get_clock()->now();
    last_odom_time_   = this->get_clock()->now();
    first_odom_       = true;
    last_v_           = 0.0;
    last_vy_          = 0.0;
    last_omega_       = 0.0;
  }

private:

  // ---------------------------------------------------------------------------
  // CALLBACK ODOMETRIA
  // Unica sorgente di predict: aggiorna lo stato ad ogni messaggio odom.
  // ---------------------------------------------------------------------------
  void odomCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);

    rclcpp::Time now(msg->header.stamp, this->get_clock()->get_clock_type());

    if (first_odom_) {
      last_update_time_ = now;
      last_odom_time_   = now;
      first_odom_       = false;
      RCLCPP_INFO(this->get_logger(), "Prima odometria ricevuta, EKF attivo.");
      return;
    }

    double dt = (now - last_update_time_).seconds();

    last_v_         = msg->twist.twist.linear.x;
    last_vy_        = msg->twist.twist.linear.y;
    last_omega_     = msg->twist.twist.angular.z;
    last_odom_time_ = now;

    if (dt > 0.0) {
      ekf_->predict(last_v_, last_vy_, last_omega_, dt);
      last_update_time_ = now;
    }
    // FIX 2: dt <= 0 (messaggi fuori ordine o duplicati) viene ignorato
    // senza crashare, ma non aggiorna last_update_time_ per non perdere il passo.
  }

  // ---------------------------------------------------------------------------
  // CALLBACK CONI
  // Esegue un predict-to-measurement prima del correct: porta lo stato EKF
  // esattamente al timestamp dei coni prima di aggiornarlo con le misure.
  // Senza questo allineamento, correct userebbe la posa sbagliata per
  // calcolare range/bearing predetti, corrompendo associazione e update.
  // ---------------------------------------------------------------------------
  void conesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);

    // FIX 5: se l'odometria non e' mai arrivata, segnalalo e aspetta.
    if (first_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Coni ricevuti ma odometria non ancora disponibile, skip.");
      return;
    }

    if (msg->markers.empty()) return;

    // FIX 4: il timestamp dei marker di pacsim NON e' affidabile — usa un
    // clock diverso dal sim time dell'odometria (livox_front pubblica con
    // stamp ~doppio rispetto a /pacsim/velocity). Usiamo il clock del nodo
    // (sim time) al momento della ricezione del messaggio.
    rclcpp::Time msg_time = this->get_clock()->now();

    // Predict-to-measurement: allinea lo stato EKF al momento di ricezione
    // dei coni. Soglia 500ms: gap maggiori indicano odom stale o problema
    // upstream, non vanno integrati ciecamente.
    double dt_cones = (msg_time - last_update_time_).seconds();
    if (dt_cones > 0.0 && dt_cones < 1.5) { //alzato il limite negativo per sim time pacsim alla loop closure
      ekf_->predict(last_v_, last_vy_, last_omega_, dt_cones);
      last_update_time_ = msg_time;
    } else if (dt_cones >= 1.5) { //soglia del warning aggiornata in accordo con la soglia sopra
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Gap temporale coni troppo grande (%.3f s), predict-to-measurement saltato.", dt_cones);
    }
    // dt_cones <= 0: messaggio in ritardo, correct comunque con posa attuale

    int m = msg->markers.size();
    Eigen::VectorXd ranges(m);
    Eigen::VectorXd bearings(m);

    int valid_count = 0;
    for (int i = 0; i < m; i++) {
      double x = msg->markers[i].pose.position.x;
      double y = msg->markers[i].pose.position.y;

      double r = std::hypot(x, y);
      double b = std::atan2(y, x);

      if (std::isnan(r) || std::isnan(b)) continue;
      if (r > 15.0 || r < 0.1) continue;

      ranges(valid_count)   = r;
      bearings(valid_count) = b;
      valid_count++;
    }

    if (valid_count > 0) {
      ranges.conservativeResize(valid_count);
      bearings.conservativeResize(valid_count);
      ekf_->correct(ranges, bearings);
    }
  }

  // ---------------------------------------------------------------------------
  // PUBBLICAZIONE RISULTATI
  // FIX 8: copia lo stato EKF fuori dal lock, poi pubblica senza tenerlo.
  // Questo evita che odomCallback e conesCallback restino bloccati per tutta
  // la durata del loop sui 250 landmark.
  // ---------------------------------------------------------------------------
  void publishSlamResults() {

    // FIX 7: breve lock solo per leggere last_odom_time_ e controllare staleness.
    {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      double odom_age = (this->get_clock()->now() - last_odom_time_).seconds();
      if (!first_odom_ && odom_age > ODOM_STALE_THRESHOLD_S) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Odometria non aggiornata da %.2f s — velocità congelata a v=%.2f omega=%.2f",
            odom_age, last_v_, last_omega_);
      }
    }

    // --- Copia stato e landmark con lock, poi pubblica senza ---
    Eigen::VectorXd state_copy;
    int n_landmarks = 0;
    std::vector<Eigen::Vector2d> landmarks_copy;

    {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      state_copy  = ekf_->getState();
      n_landmarks = ekf_->numLandmarks();
      landmarks_copy.reserve(n_landmarks);
      for (int i = 0; i < n_landmarks; i++) {
        landmarks_copy.push_back(ekf_->getLandmarkPosition(i));
      }
    }
    // Il mutex è rilasciato qui — odom e coni possono girare liberamente
    // mentre costruiamo e pubblichiamo i messaggi.

    // 1. Pubblica Posa
    geometry_msgs::msg::PoseStamped pst;
    pst.header.stamp    = this->get_clock()->now();
    pst.header.frame_id = "map";
    pst.pose.position.x = state_copy(0);
    pst.pose.position.y = state_copy(1);
    pst.pose.position.z = 0.0;

    double cy = std::cos(state_copy(2) * 0.5);
    double sy = std::sin(state_copy(2) * 0.5);
    pst.pose.orientation.x = 0.0;
    pst.pose.orientation.y = 0.0;
    pst.pose.orientation.z = sy;
    pst.pose.orientation.w = cy;
    pub_pose_->publish(pst);

    // 2. Pubblica Coni Mappati
    visualization_msgs::msg::MarkerArray markers_msg;

    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.header.frame_id = "map";
    delete_all_marker.header.stamp    = pst.header.stamp;
    delete_all_marker.ns              = "ekf_cones";
    delete_all_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
    markers_msg.markers.push_back(delete_all_marker);

    for (int i = 0; i < n_landmarks; i++) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "map";
      mk.header.stamp    = pst.header.stamp;
      mk.ns              = "ekf_cones";
      mk.id              = i;
      mk.type            = visualization_msgs::msg::Marker::CYLINDER;
      mk.action          = visualization_msgs::msg::Marker::ADD;

      mk.pose.position.x  = landmarks_copy[i](0);
      mk.pose.position.y  = landmarks_copy[i](1);
      mk.pose.position.z  = 0.0;
      mk.pose.orientation.w = 1.0;

      mk.scale.x = 0.2; mk.scale.y = 0.2; mk.scale.z = 0.5;
      mk.color.r = 0.0; mk.color.g = 0.5; mk.color.b = 1.0; mk.color.a = 1.0;
      mk.lifetime = rclcpp::Duration::from_seconds(0);

      markers_msg.markers.push_back(mk);
    }
    pub_markers_->publish(markers_msg);
  }

  // --- Membri ---
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_odom_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr           sub_cones_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr      pub_pose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  rclcpp::TimerBase::SharedPtr timer_viz_;

  std::shared_ptr<EKF_SLAM> ekf_;
  std::mutex                 ekf_mutex_;

  rclcpp::Time last_update_time_;
  rclcpp::Time last_odom_time_;
  bool         first_odom_;
  double       last_v_;
  double       last_vy_;
  double       last_omega_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // FIX sim_time: forza use_sim_time=true prima che il nodo venga costruito,
  // così get_clock() usa il sim time fin dal primo messaggio.
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides({{"use_sim_time", true}});

  rclcpp::spin(std::make_shared<EKFNode>(options));
  rclcpp::shutdown();
  return 0;
}



