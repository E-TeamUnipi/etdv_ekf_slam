#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

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
        self.pub_map_rmse = self.create_publisher(Float64, '/ekf/map_rmse', 10)
        
        self.pub_lat_p50 = self.create_publisher(Float64, '/ekf/latency_p50', 10)
        self.pub_lat_p99 = self.create_publisher(Float64, '/ekf/latency_p99', 10)
        self.pub_imu_lat_p50 = self.create_publisher(Float64, '/ekf/imu_latency_p50', 10)
        self.pub_imu_lat_p99 = self.create_publisher(Float64, '/ekf/imu_latency_p99', 10)

        # --- Subscribers ---
        self.sub_ekf = self.create_subscription(Odometry, '/ekf/odometry', self.ekf_callback, 10)
        self.sub_gt_map = self.create_subscription(Track, '/pacsim/track/landmarks', self.gt_map_callback, 10)
        self.sub_est_map = self.create_subscription(MarkerArray, '/ekf/map_cones', self.est_map_callback, 10)
        self.sub_latency = self.create_subscription(Float64, '/ekf/latency_ms', self.latency_callback, 10)
        self.sub_imu_latency = self.create_subscription(Float64, '/ekf/imu_latency_ms', self.imu_latency_callback, 10)

        # --- Buffer ---
        self.latency_buffer = []
        self.imu_latency_buffer = []
        
        # --- Variabili di stato (Posa) ---
        self.sample_count = 0
        self.sum_sq_x = 0.0
        self.sum_sq_y = 0.0
        self.sum_sq_theta = 0.0
        self.gt_map_points = []
        
        self.get_logger().info("Nodo Metriche (Posa & Mappa) avviato con successo!")

    def gt_map_callback(self, msg):
        if len(self.gt_map_points) > 0:
            return
        for lane in [msg.left_lane, msg.right_lane, msg.unknown]:
            for cone in lane:
                self.gt_map_points.append((cone.pose.pose.position.x, cone.pose.pose.position.y))
        if len(self.gt_map_points) > 0:
            self.get_logger().info(f"Mappa Ground Truth acquisita! Totale coni: {len(self.gt_map_points)}")

    def est_map_callback(self, msg):
        if len(self.gt_map_points) == 0 or len(msg.markers) == 0:
            return
        sum_sq_error = 0.0
        num_cones = len(msg.markers)
        for marker in msg.markers:
            est_x = marker.pose.position.x
            est_y = marker.pose.position.y
            min_dist_sq = float('inf')
            for gt_x, gt_y in self.gt_map_points:
                dist_sq = (est_x - gt_x)**2 + (est_y - gt_y)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
            sum_sq_error += min_dist_sq
            
        rmse_val = math.sqrt(sum_sq_error / num_cones)
        self.pub_map_rmse.publish(Float64(data=float(rmse_val)))

    def ekf_callback(self, msg):
        try:
            # Ricerca istantanea su 'car' (che coincide col cog!)
            t = self.tf_buffer.lookup_transform('map', 'car', rclpy.time.Time())
            
            error_x = msg.pose.pose.position.x - t.transform.translation.x
            error_y = msg.pose.pose.position.y - t.transform.translation.y
            
            yaw_ekf = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            yaw_true = 2.0 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
            
            raw_diff = yaw_ekf - yaw_true
            error_theta_rad = math.atan2(math.sin(raw_diff), math.cos(raw_diff))
            error_theta_deg = math.degrees(error_theta_rad)
            
            error_msg = Point(x=error_x, y=error_y, z=error_theta_deg)
            self.pub_error.publish(error_msg)
            
            self.sample_count += 1
            self.sum_sq_x += error_x**2
            self.sum_sq_y += error_y**2
            self.sum_sq_theta += error_theta_deg**2
            
            rmse_msg = Point(
                x=math.sqrt(self.sum_sq_x / self.sample_count),
                y=math.sqrt(self.sum_sq_y / self.sample_count),
                z=math.sqrt(self.sum_sq_theta / self.sample_count)
            )
            self.pub_rmse.publish(rmse_msg)
            
            true_pose = PoseStamped()
            true_pose.header.stamp = msg.header.stamp
            true_pose.header.frame_id = 'map'
            true_pose.pose.position.x = t.transform.translation.x
            true_pose.pose.position.y = t.transform.translation.y
            true_pose.pose.orientation = t.transform.rotation
            self.pub_true_pose.publish(true_pose)
            
        except TransformException as ex:
            # Stampiamo un warning senza intasare il terminale per capire se le TF saltano
            self.get_logger().warn(f"Attesa TF map->car... : {ex}", throttle_duration_sec=2.0)

    def latency_callback(self, msg):
        self.latency_buffer.append(msg.data)
        if len(self.latency_buffer) > 1000:
            self.latency_buffer.pop(0)
        if len(self.latency_buffer) >= 100:
            sorted_lats = sorted(self.latency_buffer)
            self.pub_lat_p50.publish(Float64(data=float(sorted_lats[int(0.50 * len(sorted_lats))])))
            self.pub_lat_p99.publish(Float64(data=float(sorted_lats[int(0.99 * len(sorted_lats))])))

    def imu_latency_callback(self, msg):
        self.imu_latency_buffer.append(msg.data)
        if len(self.imu_latency_buffer) > 1000:
            self.imu_latency_buffer.pop(0)
        if len(self.imu_latency_buffer) >= 100:
            sorted_lats = sorted(self.imu_latency_buffer)
            self.pub_imu_lat_p50.publish(Float64(data=float(sorted_lats[int(0.50 * len(sorted_lats))])))
            self.pub_imu_lat_p99.publish(Float64(data=float(sorted_lats[int(0.99 * len(sorted_lats))])))

def main():
    rclpy.init()
    node = TFToPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()