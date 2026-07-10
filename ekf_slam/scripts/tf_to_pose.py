#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class TFToPoseNode(Node):
    def __init__(self):
        super().__init__('tf_to_pose_bridge')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher originali
        self.pub_true_pose = self.create_publisher(PoseStamped, '/pacsim/true_pose', 10)
        self.pub_error = self.create_publisher(Point, '/ekf/error', 10)
        
        # NUOVO: Publisher per l'RMSE
        self.pub_rmse = self.create_publisher(Point, '/ekf/rmse', 10)
        
        self.sub_ekf = self.create_subscription(Odometry, '/ekf/odometry', self.ekf_callback, 10)
        
        # --- Variabili di stato per accumulare l'RMSE ---
        self.sample_count = 0
        self.sum_sq_x = 0.0
        self.sum_sq_y = 0.0
        self.sum_sq_theta = 0.0
        
        self.get_logger().info("Bridge TF, Calcolatore di Errore e RMSE avviati!")

    def ekf_callback(self, msg):
        try:
            t = self.tf_buffer.lookup_transform('map', 'car', rclpy.time.Time())
            
            # --- 1. CALCOLO ERRORE ISTANTANEO ---
            error_x = msg.pose.pose.position.x - t.transform.translation.x
            error_y = msg.pose.pose.position.y - t.transform.translation.y
            
            yaw_ekf = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            yaw_true = 2.0 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
            
            raw_diff = yaw_ekf - yaw_true
            error_theta_rad = math.atan2(math.sin(raw_diff), math.cos(raw_diff))
            error_theta_deg = math.degrees(error_theta_rad)
            
            # Pubblica errore istantaneo
            error_msg = Point()
            error_msg.x = error_x
            error_msg.y = error_y
            error_msg.z = error_theta_deg
            self.pub_error.publish(error_msg)
            
            # --- 2. CALCOLO RMSE ---
            self.sample_count += 1
            
            # Accumula i quadrati degli errori
            self.sum_sq_x += error_x**2
            self.sum_sq_y += error_y**2
            self.sum_sq_theta += error_theta_deg**2
            
            # Calcola la radice della media
            rmse_msg = Point()
            rmse_msg.x = math.sqrt(self.sum_sq_x / self.sample_count)
            rmse_msg.y = math.sqrt(self.sum_sq_y / self.sample_count)
            rmse_msg.z = math.sqrt(self.sum_sq_theta / self.sample_count)
            
            self.pub_rmse.publish(rmse_msg)
            
            # --- 3. PUBBLICAZIONE VERA POSA ---
            true_pose = PoseStamped()
            true_pose.header.stamp = msg.header.stamp
            true_pose.header.frame_id = 'map'
            true_pose.pose.position.x = t.transform.translation.x
            true_pose.pose.position.y = t.transform.translation.y
            true_pose.pose.orientation = t.transform.rotation
            self.pub_true_pose.publish(true_pose)
            
        except TransformException as ex:
            pass

def main():
    rclpy.init()
    node = TFToPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()