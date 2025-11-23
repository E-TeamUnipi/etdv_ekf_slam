/**
 * @file circular_controller_feedback_node.cpp
 * @brief ROS2 controller con feedback per percorso circolare
 * 
 * Questo controller usa il feedback dalla posizione del veicolo per
 * correggere in tempo reale l'angolo di sterzo e mantenere il raggio costante.
 * 
 * Topics pubblicati:
 *   - /pacsim/steering_setpoint (pacsim/msg/StampedScalar): angolo del volante
 *   - /pacsim/torques_max (pacsim/msg/Wheels): torque massimo
 * 
 * Topics sottoscritti:
 *   - /tf (tf2_msgs/msg/TFMessage): per ottenere la posizione del veicolo
 */

#include "rclcpp/rclcpp.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cmath>

class CircularControllerFeedback : public rclcpp::Node
{
public:
    CircularControllerFeedback() : Node("circular_controller_feedback"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Parametri base
        this->declare_parameter("circle_radius", 13.5);
        this->declare_parameter("target_velocity", 3.0);  // Ridotta da 5.0 a 3.0
        this->declare_parameter("torque_per_wheel", 12.0);  // Ridotto da 15.0 a 12.0
        this->declare_parameter("warmup_time", 3.0);  // Aumentato da 2.0 a 3.0
        this->declare_parameter("acceleration_time", 4.0);  // Tempo di applicazione torque
        this->declare_parameter("publish_rate", 50.0);
        
        // Parametri veicolo
        this->declare_parameter("wheelbase_front", 0.78);
        this->declare_parameter("wheelbase_rear", 0.72);
        this->declare_parameter("inner_steering_ratio", 0.255625);
        this->declare_parameter("outer_steering_ratio", 0.20375);
        
        // Parametri feedback PID
        this->declare_parameter("kp", 3.5);  // Guadagno proporzionale
        this->declare_parameter("ki", 0.5);  // Guadagno integrale
        this->declare_parameter("kd", 0.8);  // Guadagno derivativo
        this->declare_parameter("integral_max", 0.3);  // Limite anti-windup per l'integrale
        this->declare_parameter("center_x", 0.0);  // Centro del cerchio
        this->declare_parameter("center_y", -13.5);  // Centro del cerchio
        
        // Parametri controllo velocità
        this->declare_parameter("min_velocity", 1.0);  // Velocità minima: sotto questa riaccende torque
        this->declare_parameter("max_velocity", 2.0);  // Velocità massima: sopra questa spegne torque
        
        // Lettura parametri
        target_radius_ = this->get_parameter("circle_radius").as_double();
        target_velocity_ = this->get_parameter("target_velocity").as_double();
        torque_per_wheel_ = this->get_parameter("torque_per_wheel").as_double();
        warmup_time_ = this->get_parameter("warmup_time").as_double();
        acceleration_time_ = this->get_parameter("acceleration_time").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        double lf = this->get_parameter("wheelbase_front").as_double();
        double lr = this->get_parameter("wheelbase_rear").as_double();
        double outer_steering_ratio = this->get_parameter("outer_steering_ratio").as_double();
        
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        integral_max_ = this->get_parameter("integral_max").as_double();
        center_x_ = this->get_parameter("center_x").as_double();
        center_y_ = this->get_parameter("center_y").as_double();
        
        min_velocity_ = this->get_parameter("min_velocity").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        
        // Calcolo angolo base
        double wheelbase = lf + lr;
        double delta_wheel = std::atan(wheelbase / target_radius_);
        
        // Fattore di correzione empirico per compensare effetti dinamici
        // Il modello cinematico di Ackermann prevede un raggio più stretto di quello reale
        // a causa delle forze laterali. Riduciamo lo sterzo dell'8% circa.
        double steering_compensation = 0.92;
        
        base_steering_angle_ = -(delta_wheel / outer_steering_ratio) * steering_compensation;
        
        RCLCPP_INFO(this->get_logger(), "=== Circular Controller with Feedback ===");
        RCLCPP_INFO(this->get_logger(), "Target radius: %.2f m", target_radius_);
        RCLCPP_INFO(this->get_logger(), "Target velocity: %.2f m/s", target_velocity_);
        RCLCPP_INFO(this->get_logger(), "Velocity control: %.2f - %.2f m/s", min_velocity_, max_velocity_);
        RCLCPP_INFO(this->get_logger(), "Circle center: (%.2f, %.2f)", center_x_, center_y_);
        RCLCPP_INFO(this->get_logger(), "Base steering: %.4f rad", base_steering_angle_);
        RCLCPP_INFO(this->get_logger(), "PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "Torque control: velocity-based (adaptive)");
        RCLCPP_INFO(this->get_logger(), "=======================================");
        
        // Publishers
        steering_pub_ = this->create_publisher<pacsim::msg::StampedScalar>(
            "/pacsim/steering_setpoint", 10);
        torques_max_pub_ = this->create_publisher<pacsim::msg::Wheels>(
            "/pacsim/torques_max", 10);
        
        // Subscriber per velocità
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/pacsim/velocity", 10,
            std::bind(&CircularControllerFeedback::velocityCallback, this, std::placeholders::_1));
        
        // Timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&CircularControllerFeedback::timerCallback, this));
        
        start_time_ = this->now();
    }

private:
    void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        // Calcola velocità scalare da vx e vy
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_velocity_ = std::sqrt(vx * vx + vy * vy);
    }
    
    void timerCallback()
    {
        auto current_time = this->now();
        double elapsed = (current_time - start_time_).seconds();
        
        // Calcola torque basato sulla velocità effettiva (dopo il warmup)
        double torque_multiplier = 0.0;
        
        if (elapsed < warmup_time_) {
            // Durante warmup: rampa iniziale indipendente dalla velocità
            torque_multiplier = elapsed / warmup_time_;
        } else {
            // Dopo warmup: controllo basato su velocità con ISTERESI
            
            // Se torque è attualmente ON e velocità > max_velocity → SPEGNI
            if (torque_enabled_ && current_velocity_ > max_velocity_) {
                torque_enabled_ = false;
                RCLCPP_INFO(this->get_logger(), 
                    "🔴 TORQUE OFF: v=%.2f m/s > %.2f m/s (max)", 
                    current_velocity_, max_velocity_);
            }
            // Se torque è attualmente OFF e velocità < min_velocity → ACCENDI
            else if (!torque_enabled_ && current_velocity_ < min_velocity_) {
                torque_enabled_ = true;
                RCLCPP_INFO(this->get_logger(), 
                    "🟢 TORQUE ON: v=%.2f m/s < %.2f m/s (min)", 
                    current_velocity_, min_velocity_);
            }
            
            torque_multiplier = torque_enabled_ ? 1.0 : 0.0;
        }
        
        // Calcola correzione basata sulla posizione
        double steering_angle = base_steering_angle_;
        
        if (elapsed > warmup_time_) {
            try {
                // Ottieni posizione dal TF
                geometry_msgs::msg::TransformStamped transform;
                transform = tf_buffer_.lookupTransform("map", "car", tf2::TimePointZero);
                
                double x = transform.transform.translation.x;
                double y = transform.transform.translation.y;
                
                // Calcola distanza dal centro
                double dx = x - center_x_;
                double dy = y - center_y_;
                double current_radius = std::sqrt(dx * dx + dy * dy);
                
                // Errore radiale
                double radius_error = current_radius - target_radius_;
                
                // Controllore PID per correzione traiettoria
                // P: Proporzionale all'errore attuale
                // I: Integrale dell'errore nel tempo (elimina errore stazionario)
                // D: Derivata dell'errore (smorzamento, riduce oscillazioni)
                
                // Calcola dt per integrale e derivata
                double dt = elapsed - last_update_time_;
                if (dt > 0.001) {  // Evita divisioni per zero
                    
                    // Termine Proporzionale
                    double P = kp_ * radius_error;
                    
                    // Termine Integrale (con anti-windup)
                    integral_error_ += radius_error * dt;
                    // Anti-windup: limita l'integrale
                    integral_error_ = std::max(-integral_max_, std::min(integral_max_, integral_error_));
                    double I = ki_ * integral_error_;
                    
                    // Termine Derivativo
                    double error_derivative = (radius_error - previous_error_) / dt;
                    double D = kd_ * error_derivative;
                    
                    // Correzione PID totale (normalizzata per il raggio e sterzo base)
                    double pid_output = (P + I + D) * abs(base_steering_angle_) / target_radius_;
                    
                    // Applica correzione
                    // Se err < 0 (troppo dentro): pid_output negativo → sottrarre negativo = sommare → allarga ✓
                    // Se err > 0 (troppo fuori): pid_output positivo → sottrarre positivo → stringe ✓
                    steering_angle = base_steering_angle_ - pid_output;
                    
                    // Aggiorna per prossima iterazione
                    previous_error_ = radius_error;
                    last_update_time_ = elapsed;
                    
                    // Log ogni 2 secondi
                    if (static_cast<int>(elapsed * 2) % 4 == 0 && 
                        elapsed - last_log_time_ >= 1.9) {
                        const char* torque_status = torque_enabled_ ? "ON 🟢" : "OFF 🔴";
                        RCLCPP_INFO(this->get_logger(), 
                            "t:%.1fs | v:%.2f m/s | R:%.2fm (err:%+.2fm) | PID: P=%.3f I=%.3f D=%.3f | Steer:%.4f rad | Torque:%s", 
                            elapsed, current_velocity_, current_radius, radius_error, 
                            P, I, D, steering_angle, torque_status);
                        last_log_time_ = elapsed;
                    }
                }
                
            } catch (const tf2::TransformException &ex) {
                // Se non riesco a leggere il TF, uso l'angolo base
                if (elapsed - last_warn_time_ >= 5.0) {
                    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                    last_warn_time_ = elapsed;
                }
            }
        } else {
            // Durante warmup: rampa graduale dello sterzo
            if (elapsed < 0.5) {
                steering_angle = 0.0;
            } else {
                double steering_mult = (elapsed - 0.5) / (warmup_time_ - 0.5);
                steering_angle = base_steering_angle_ * steering_mult;
            }
        }
        
        publishSteering(steering_angle, current_time);
        publishTorques(torque_per_wheel_ * torque_multiplier, current_time);
    }
    
    void publishSteering(double angle, const rclcpp::Time& stamp)
    {
        pacsim::msg::StampedScalar msg;
        msg.stamp = stamp;
        msg.value = angle;
        steering_pub_->publish(msg);
    }
    
    void publishTorques(double torque, const rclcpp::Time& stamp)
    {
        pacsim::msg::Wheels msg;
        msg.stamp = stamp;
        msg.fl = torque;
        msg.fr = torque;
        msg.rl = torque;
        msg.rr = torque;
        torques_max_pub_->publish(msg);
    }
    
    // Publishers
    rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
    rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torques_max_pub_;
    
    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    // Parametri
    double target_radius_;
    double target_velocity_;
    double torque_per_wheel_;
    double warmup_time_;
    double acceleration_time_;
    double kp_;  // Guadagno proporzionale PID
    double ki_;  // Guadagno integrale PID
    double kd_;  // Guadagno derivativo PID
    double integral_max_;  // Limite anti-windup
    double center_x_;
    double center_y_;
    double min_velocity_;
    double max_velocity_;
    
    // Stati
    double base_steering_angle_;
    double current_velocity_ = 0.0;
    bool torque_enabled_ = true;  // Inizia con torque abilitato
    double last_log_time_ = -10.0;
    double last_warn_time_ = -10.0;
    
    // Stati PID
    double integral_error_ = 0.0;
    double previous_error_ = 0.0;
    double last_update_time_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CircularControllerFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
