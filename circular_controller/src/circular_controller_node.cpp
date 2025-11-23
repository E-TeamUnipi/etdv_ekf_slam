/**
 * @file circular_controller_node.cpp
 * @brief ROS2 controller per far seguire alla macchina un percorso circolare preciso
 * 
 * Questo nodo pubblica comandi di sterzo e torque per far muovere il veicolo
 * lungo una traiettoria circolare di raggio specificato.
 * 
 * Topics pubblicati:
 *   - /pacsim/steering_setpoint (pacsim/msg/StampedScalar): angolo del volante
 *   - /pacsim/torques_max (pacsim/msg/Wheels): torque massimo per ogni ruota
 * 
 * Parametri:
 *   - circle_radius: raggio del cerchio in metri (default: 13.5)
 *   - target_velocity: velocità target in m/s (default: 5.0)
 *   - torque_per_wheel: torque per ruota in Nm (default: 15.0)
 *   - warmup_time: tempo di ramp-up iniziale in secondi (default: 2.0)
 */

#include "rclcpp/rclcpp.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include <cmath>

class CircularController : public rclcpp::Node
{
public:
    CircularController() : Node("circular_controller")
    {
        // Dichiarazione parametri
        this->declare_parameter("circle_radius", 13.5);
        this->declare_parameter("target_velocity", 3.0);  // Ridotta da 5.0
        this->declare_parameter("torque_per_wheel", 12.0);  // Ridotto da 15.0
        this->declare_parameter("warmup_time", 3.0);  // Aumentato da 2.0
        this->declare_parameter("acceleration_time", 4.0);  // NUOVO: tempo applicazione torque
        this->declare_parameter("publish_rate", 50.0);  // Hz
        
        // Parametri veicolo (dal modello)
        this->declare_parameter("wheelbase_front", 0.78);   // lf
        this->declare_parameter("wheelbase_rear", 0.72);    // lr
        this->declare_parameter("inner_steering_ratio", 0.255625);
        this->declare_parameter("outer_steering_ratio", 0.20375);
        
        // Fattore di compensazione per ridurre la tendenza a stringere
        // Valori < 1.0 riducono l'angolo di sterzo (raggio più largo)
        // Valori > 1.0 aumentano l'angolo di sterzo (raggio più stretto)
        this->declare_parameter("steering_compensation_factor", 0.92);
        
        // Lettura parametri
        circle_radius_ = this->get_parameter("circle_radius").as_double();
        target_velocity_ = this->get_parameter("target_velocity").as_double();
        torque_per_wheel_ = this->get_parameter("torque_per_wheel").as_double();
        warmup_time_ = this->get_parameter("warmup_time").as_double();
        acceleration_time_ = this->get_parameter("acceleration_time").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        double lf = this->get_parameter("wheelbase_front").as_double();
        double lr = this->get_parameter("wheelbase_rear").as_double();
        inner_steering_ratio_ = this->get_parameter("inner_steering_ratio").as_double();
        double steering_compensation = this->get_parameter("steering_compensation_factor").as_double();
        
        // Calcoli cinematici
        double wheelbase = lf + lr;  // passo del veicolo
        
        // Angolo di sterzo delle ruote (modello bicicletta)
        double delta_wheel = std::atan(wheelbase / circle_radius_);
        
        // Angolo del volante (considerando il rapporto di sterzo interno)
        // NEGATIVO per curva a destra (senso orario)
        // Per angoli negativi (curva a destra) usiamo outer_steering_ratio
        double outer_steering_ratio = this->get_parameter("outer_steering_ratio").as_double();
        steering_wheel_angle_ = -delta_wheel / outer_steering_ratio * steering_compensation;
        
        RCLCPP_INFO(this->get_logger(), "=== Circular Controller Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Circle radius: %.2f m", circle_radius_);
        RCLCPP_INFO(this->get_logger(), "Target velocity: %.2f m/s", target_velocity_);
        RCLCPP_INFO(this->get_logger(), "Wheelbase: %.3f m", wheelbase);
        RCLCPP_INFO(this->get_logger(), "Wheel steering angle: %.4f rad (%.2f°)", 
                    delta_wheel, delta_wheel * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Steering wheel angle: %.4f rad (%.2f°)", 
                    steering_wheel_angle_, steering_wheel_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Torque per wheel: %.2f Nm", torque_per_wheel_);
        RCLCPP_INFO(this->get_logger(), "Acceleration phase: 0 - %.1f s", acceleration_time_);
        RCLCPP_INFO(this->get_logger(), "Inertia phase: %.1f s onwards (no torque)", acceleration_time_);
        RCLCPP_INFO(this->get_logger(), "Expected angular velocity: %.4f rad/s", 
                    target_velocity_ / circle_radius_);
        RCLCPP_INFO(this->get_logger(), "Circle circumference: %.2f m", 
                    2.0 * M_PI * circle_radius_);
        RCLCPP_INFO(this->get_logger(), "Expected lap time: %.2f s", 
                    2.0 * M_PI * circle_radius_ / target_velocity_);
        RCLCPP_INFO(this->get_logger(), "======================================");
        
        // Publisher
        steering_pub_ = this->create_publisher<pacsim::msg::StampedScalar>(
            "/pacsim/steering_setpoint", 10);
        torques_max_pub_ = this->create_publisher<pacsim::msg::Wheels>(
            "/pacsim/torques_max", 10);
        
        // Timer per pubblicazione periodica
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&CircularController::timerCallback, this));
        
        start_time_ = this->now();
    }

private:
    void timerCallback()
    {
        auto current_time = this->now();
        double elapsed = (current_time - start_time_).seconds();
        
        // Calcola torque basato sul tempo
        double torque_multiplier = 0.0;
        
        if (elapsed < acceleration_time_) {
            // Fase di accelerazione
            if (elapsed < warmup_time_) {
                // Ramp-up lineare del torque
                torque_multiplier = elapsed / warmup_time_;
            } else {
                // Dopo warmup: torque pieno fino a acceleration_time
                torque_multiplier = 1.0;
            }
        } else {
            // Fase di inerzia: NESSUN torque
            torque_multiplier = 0.0;
        }
        
        // Gestione dello sterzo
        if (elapsed < warmup_time_) {
            if (elapsed < 0.5) {
                // Primi 0.5s: solo torque, niente sterzo (accelerazione dritta)
                publishSteering(0.0, current_time);
            } else {
                // Dopo 0.5s: inizia gradualmente lo sterzo
                double steering_multiplier = (elapsed - 0.5) / (warmup_time_ - 0.5);
                publishSteering(steering_wheel_angle_ * steering_multiplier, current_time);
            }
        } else {
            // Dopo warmup: sterzo completo
            publishSteering(steering_wheel_angle_, current_time);
        }
        
        publishTorques(torque_per_wheel_ * torque_multiplier, current_time);
        
        // Log periodico ogni 5 secondi
        if (static_cast<int>(elapsed) % 5 == 0 && 
            elapsed - last_log_time_ >= 4.9) {
            const char* phase = (elapsed < acceleration_time_) ? "ACCEL" : "INERTIA";
            RCLCPP_INFO(this->get_logger(), 
                "[%s] t:%.1fs | Torque: %.1f%% | Steering: %.4f rad", 
                phase, elapsed, torque_multiplier * 100.0,
                (elapsed < 0.5) ? 0.0 : steering_wheel_angle_);
            last_log_time_ = elapsed;
        }
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
        // Torque uguale su tutte le ruote (AWD bilanciato)
        msg.fl = torque;
        msg.fr = torque;
        msg.rl = torque;
        msg.rr = torque;
        torques_max_pub_->publish(msg);
    }
    
    // Publishers
    rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
    rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torques_max_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    // Parametri
    double circle_radius_;
    double target_velocity_;
    double torque_per_wheel_;
    double warmup_time_;
    double acceleration_time_;
    double inner_steering_ratio_;
    
    // Stati calcolati
    double steering_wheel_angle_;
    double last_log_time_ = -10.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CircularController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
