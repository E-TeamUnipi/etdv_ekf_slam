#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

    double nv  = this->get_parameter("process_noise_v").as_double();
    double nvy = this->get_parameter("process_noise_vy").as_double();
    double nw  = this->get_parameter("process_noise_omega").as_double();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    ekf_->setProcessNoise(nv, nvy, nw);

    // Sottoscrizioni
    sub_vel_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10,
        std::bind(&EKFPoseNode::velocityCallback, this, _1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/pacsim/imu/cog_imu", 10,
        std::bind(&EKFPoseNode::imuCallback, this, _1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose_only", 10);

    timer_viz_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(0.05), // 20 Hz
        std::bind(&EKFPoseNode::publishPose, this));

    first_odom_ = true;
    last_v_  = 0.0;
    last_vy_ = 0.0;
    last_omega_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "EKF Pose-Only Node initialized.");
  }

private:
  void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    last_v_  = msg->twist.twist.linear.x;
    last_vy_ = msg->twist.twist.linear.y;
  }

  // Sostituisci la parte finale della imuCallback così:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);

    rclcpp::Time now(msg->header.stamp, this->get_clock()->get_clock_type());
    if (first_odom_) { last_update_time_ = now; first_odom_ = false; return; }

    double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) return;

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double w  = msg->angular_velocity.z;

    // FASE 1: Predict
    ekf_->predict(ax, ay, dt);

    // FASE 2: Correct e Debug
    double y = 0.0;
    double k = 0.0;
    ekf_->correctGyro(w, y, k); 
    
    // LOGGING PULITO DAL NODO
    static int counter = 0;
    if (counter++ % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), 
            "DEBUG Gyro -> Innovazione(y): %.4f, KalmanGain(K): %.6f", 
            y, k);
    }

    last_update_time_ = now;
}

  void publishPose() {
    if (first_odom_) return;

    // CORREZIONE: Usa VectorXd perché lo stato ora è 6D
    Eigen::VectorXd state; 
    {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      state = ekf_->getState();
    }

    // Assicurati che lo stato sia pronto (evita pubblicare se è tutto zero all'inizio)
    if (state.size() < 6) return; 

    geometry_msgs::msg::PoseStamped pst;
    pst.header.stamp    = this->get_clock()->now();
    pst.header.frame_id = "map"; 
    
    // Ora il tuo stato ha 6 elementi, i primi 3 sono [x, y, theta]
    pst.pose.position.x = state(0);
    pst.pose.position.y = state(1);
    pst.pose.position.z = 0.0;

    double cy = std::cos(state(2) * 0.5);
    double sy = std::sin(state(2) * 0.5);
    pst.pose.orientation.x = 0.0;
    pst.pose.orientation.y = 0.0;
    pst.pose.orientation.z = sy;
    pst.pose.orientation.w = cy;

    pub_pose_->publish(pst);

    // Dopo aver pubblicato pst (PoseStamped)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = pst.header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "ekf_estimate"; // Questo è il nome che vedrai in Foxglove

    t.transform.translation.x = state(0);
    t.transform.translation.y = state(1);
    t.transform.translation.z = 0.0;

    // Converti l'orientazione (quaternione)
    t.transform.rotation = pst.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::TimerBase::SharedPtr timer_viz_;


  std::unique_ptr<tf2_ros::TransformBroadcaster>    tf_broadcaster_;
  std::shared_ptr<EKFPose>                          ekf_;
  std::mutex                                        ekf_mutex_;
  rclcpp::Time                                      last_update_time_;
  
  bool   first_odom_;
  double last_v_;
  double last_vy_;
  double last_omega_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides({{"use_sim_time", true}});
  rclcpp::spin(std::make_shared<EKFPoseNode>(options));
  rclcpp::shutdown();
  return 0;
}