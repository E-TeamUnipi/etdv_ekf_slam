#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

# Import necessari per la mappa
from pacsim.msg import Track
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64

class TFToPoseNode(Node):
    def __init__(self):
        super().__init__('metrics_evaluator_node')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Publishers ---
        self.pub_true_pose = self.create_publisher(PoseStamped, '/pacsim/true_pose', 10)
        self.pub_error = self.create_publisher(Point, '/ekf/error', 10)
        self.pub_rmse = self.create_publisher(Point, '/ekf/rmse', 10)
        
        # NUOVO: Publisher per l'RMSE della mappa
        self.pub_map_rmse = self.create_publisher(Float64, '/ekf/map_rmse', 10)
        
        # --- Subscribers ---
        self.sub_ekf = self.create_subscription(Odometry, '/ekf/odometry', self.ekf_callback, 10)
        
        # NUOVO: Subscribers per le mappe
        self.sub_gt_map = self.create_subscription(Track, '/pacsim/track/landmarks', self.gt_map_callback, 10)
        self.sub_est_map = self.create_subscription(MarkerArray, '/ekf/map_cones', self.est_map_callback, 10)
        self.sub_latency = self.create_subscription(Float64, '/ekf/latency_ms', self.latency_callback, 10)
        self.pub_lat_p50 = self.create_publisher(Float64, '/ekf/latency_p50', 10)
        self.pub_lat_p99 = self.create_publisher(Float64, '/ekf/latency_p99', 10)

        # Buffer per la finestra scorrevole (ultimi 1000 campioni)
        self.latency_buffer = []
        # --- Variabili di stato (Posa) ---
        self.sample_count = 0
        self.sum_sq_x = 0.0
        self.sum_sq_y = 0.0
        self.sum_sq_theta = 0.0
        
        self.gt_map_points = []
        
        self.get_logger().info("Nodo Metriche (Posa & Mappa) avviato con successo!")

    # ==========================================
    # CALLBACK: MAPPA GROUND TRUTH
    # ==========================================
    def gt_map_callback(self, msg):
        # Se abbiamo già caricato la mappa, ignoriamo
        if len(self.gt_map_points) > 0:
            return
            
        # Estraiamo i coni dalle liste
        for lane in [msg.left_lane, msg.right_lane, msg.unknown]:
            for cone in lane:
                self.gt_map_points.append((cone.pose.pose.position.x, cone.pose.pose.position.y))
                
        if len(self.gt_map_points) > 0:
            self.get_logger().info(f"Mappa Ground Truth acquisita! Totale coni: {len(self.gt_map_points)}")

    # ==========================================
    # CALLBACK: MAPPA STIMATA DALL'EKF
    # ==========================================
    def est_map_callback(self, msg):
        # Se non abbiamo ancora il Ground Truth o la mappa EKF è vuota, usciamo
        if len(self.gt_map_points) == 0 or len(msg.markers) == 0:
            return
            
        sum_sq_error = 0.0
        num_cones = len(msg.markers)
        
        for marker in msg.markers:
            est_x = marker.pose.position.x
            est_y = marker.pose.position.y
            
            min_dist_sq = float('inf')
            
            # Ricerca del Nearest Neighbor (forza bruta, ma va benissimo per 2Hz)
            for gt_x, gt_y in self.gt_map_points:
                dist_sq = (est_x - gt_x)**2 + (est_y - gt_y)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    
            sum_sq_error += min_dist_sq
            
        # Calcolo statistico dell'RMSE
        rmse_val = math.sqrt(sum_sq_error / num_cones)
        
        # Pubblicazione
        rmse_msg = Float64()
        rmse_msg.data = float(rmse_val)
        self.pub_map_rmse.publish(rmse_msg)

    # ==========================================
    # CALLBACK: POSA ODOMETRIA
    # ==========================================
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
            
            # --- 2. CALCOLO RMSE POSA ---
            self.sample_count += 1
            
            self.sum_sq_x += error_x**2
            self.sum_sq_y += error_y**2
            self.sum_sq_theta += error_theta_deg**2
            
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

    # ==========================================
    # CALLBACK: CALCOLO PERCENTILI LATENZA
    # ==========================================
    def latency_callback(self, msg):
        self.latency_buffer.append(msg.data)
        
        # Manteniamo solo gli ultimi 1000 campioni (circa 100 secondi a 10Hz) 
        # per avere una CDF responsiva all'andamento attuale
        if len(self.latency_buffer) > 1000:
            self.latency_buffer.pop(0)
            
        # Aspettiamo di avere almeno 100 campioni per fare una statistica sensata
        if len(self.latency_buffer) >= 100:
            # Ordiniamo l'array dal più veloce al più lento
            sorted_lats = sorted(self.latency_buffer)
            
            # Peschiamo gli indici
            idx_p50 = int(0.50 * len(sorted_lats))
            idx_p99 = int(0.99 * len(sorted_lats))
            
            # Pubblichiamo
            msg_p50 = Float64()
            msg_p50.data = float(sorted_lats[idx_p50])
            self.pub_lat_p50.publish(msg_p50)
            
            msg_p99 = Float64()
            msg_p99.data = float(sorted_lats[idx_p99])
            self.pub_lat_p99.publish(msg_p99)

def main():
    rclpy.init()
    node = TFToPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()