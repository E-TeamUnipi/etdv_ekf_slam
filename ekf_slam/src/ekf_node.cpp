#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/pacsim/imu/cog_imu", 10, std::bind(&EKFPoseNode::imuCallback, this, _1));
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose_only", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odometry", 10);

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

    ekf_->predict(msg->linear_acceleration.x, msg->linear_acceleration.y, dt);
    double y, k;
    ekf_->correctGyro(msg->angular_velocity.z, y, k); 
    last_update_time_ = now;
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
}

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
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