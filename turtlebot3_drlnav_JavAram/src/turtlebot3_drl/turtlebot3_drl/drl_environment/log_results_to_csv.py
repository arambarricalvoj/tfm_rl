#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan, BatteryState
from irobot_create_msgs.msg import HazardDetectionVector
from std_msgs.msg import Float32 # Importamos el nuevo mensaje

import math
import csv
import time

class RealExperimentEvaluator(Node):
    def __init__(self):
        super().__init__('real_experiment_evaluator')
        
        self.start_time = time.time()
        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None
        self.collision_count = 0
        self.min_obstacle_dist = 0.0
        self.battery_percentage = 1.0
        self.map_explored_percent = 0.0
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.accumulated_jerk = 0.0

        self.csv_filename = f"real_eval_metrics_{int(self.start_time)}.csv"
        self.init_csv()

        # --- SUSCRIPTORES ---
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_stable', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.hazard_sub = self.create_subscription(HazardDetectionVector, '/vin/hazard_detection', self.hazard_callback, 10)
        self.battery_sub = self.create_subscription(BatteryState, '/vin/battery_state', self.battery_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/vin/cmd_vel', self.velocity_callback, 10)
        
        # --- EL NUEVO SUSCRIPTOR DEL PORCENTAJE ---
        self.explored_sub = self.create_subscription(Float32, '/drl/exploration_percent', self.explored_callback, 10)

        self.timer = self.create_timer(1.0, self.log_to_csv)
        self.get_logger().info(f"Evaluador Optimizado Activo. Guardando en: {self.csv_filename}")

    def init_csv(self):
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time_Elapsed_s', 'Explored_Percent', 'Distance_Traveled_m', 'Collisions', 'Min_Obstacle_Dist_m', 'Battery_Percent', 'Accumulated_Velocity_Jerk'])

    def explored_callback(self, msg):
        self.map_explored_percent = msg.data

    def pose_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        if self.last_x is not None and self.last_y is not None:
            self.total_distance += math.hypot(current_x - self.last_x, current_y - self.last_y)
        self.last_x = current_x
        self.last_y = current_y

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0.0]
        if valid_ranges:
            self.min_obstacle_dist = min(valid_ranges)

    def hazard_callback(self, msg):
        if len(msg.detections) > 1:
            self.collision_count += 1

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage

    def velocity_callback(self, msg):
        d_lin = abs(msg.linear.x - self.last_linear_vel)
        d_ang = abs(msg.angular.z - self.last_angular_vel)
        self.accumulated_jerk += (d_lin + d_ang)
        self.last_linear_vel = msg.linear.x
        self.last_angular_vel = msg.angular.z

    def log_to_csv(self):
        elapsed = time.time() - self.start_time
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([round(elapsed, 2), round(self.map_explored_percent, 2), round(self.total_distance, 3), self.collision_count, round(self.min_obstacle_dist, 3), round(self.battery_percentage * 100, 1), round(self.accumulated_jerk, 3)])

def main(args=None):
    rclpy.init(args=args)
    node = RealExperimentEvaluator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()