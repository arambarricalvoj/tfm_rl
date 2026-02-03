#!/usr/bin/env python3
import math
import time
import copy
import numpy as np
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

# ====== Constantes (ajusta si quieres a settings.py) ======
NUM_SCAN_SAMPLES = 40
LIDAR_DISTANCE_CAP = 3.5
SPEED_LINEAR_MAX = 0.22
SPEED_ANGULAR_MAX = 2.84
THREHSOLD_GOAL = 0.15
MAX_GOAL_DISTANCE = 5.0  # para normalizar distancia

# ====== Actor DDPG ======
class Actor(nn.Module):
    def __init__(self, state_size=44, action_size=2, hidden_size=512):
        super().__init__()
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)

    def forward(self, x):
        x = torch.relu(self.fa1(x))
        x = torch.relu(self.fa2(x))
        return torch.tanh(self.fa3(x))


class DRLNavigator(Node):
    def __init__(self):
        super().__init__('drl_navigator')

        # Callback group reentrante para que execute + odom + scan convivan
        self.cb_group = ReentrantCallbackGroup()

        # Modelo
        self.device = torch.device("cpu")
        self.actor = Actor(44, 2, 512).to(self.device)
        self.model_path = "/home/javierac/turtlebot3_drlnav_ws/src/turtlebot3_drl/model/f6ca27706a14/ddpg_0_stage_9/actor_stage9_episode15000.pt"
        self.actor.load_state_dict(torch.load(self.model_path, map_location=self.device))
        self.actor.eval()
        self.get_logger().info(f"Loaded DRL model: {self.model_path}")

        # Estado
        self.scan_ranges = [1.0] * NUM_SCAN_SAMPLES
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.goal_distance = MAX_GOAL_DISTANCE
        self.goal_angle = 0.0
        self.prev_action = [0.0, 0.0]

        # Subs y pubs
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10, callback_group=self.cb_group
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Action server
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

    # ====== Callbacks de sensores ======
    def scan_callback(self, msg: LaserScan):
        for i in range(min(NUM_SCAN_SAMPLES, len(msg.ranges))):
            r = float(msg.ranges[i])
            r = np.clip(r, 0.0, LIDAR_DISTANCE_CAP)
            self.scan_ranges[i] = r / LIDAR_DISTANCE_CAP

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        import numpy as np
        np.float = float  # parche para tf_transformations
        from tf_transformations import euler_from_quaternion
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_heading = yaw

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        self.goal_distance = math.sqrt(dx*dx + dy*dy)
        heading_to_goal = math.atan2(dy, dx)
        self.goal_angle = heading_to_goal - self.robot_heading

        while self.goal_angle > math.pi:
            self.goal_angle -= 2 * math.pi
        while self.goal_angle < -math.pi:
            self.goal_angle += 2 * math.pi

    # ====== Callbacks del action server ======
    def goal_callback(self, goal_request):
        self.get_logger().info("New goal received")
        pose = goal_request.pose.pose
        self.goal_x = pose.position.x
        self.goal_y = pose.position.y
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal cancel requested")
        self.cmd_pub.publish(Twist())
        return CancelResponse.ACCEPT

    # ====== Construcción del estado (igual que get_state) ======
    def build_state(self):
        state = copy.deepcopy(self.scan_ranges)  # 40
        gd_norm = float(np.clip(self.goal_distance / MAX_GOAL_DISTANCE, 0.0, 1.0))
        ga_norm = float(self.goal_angle / math.pi)  # [-1, 1]
        state.append(gd_norm)
        state.append(ga_norm)
        state.append(float(self.prev_action[0]))  # acción lineal previa
        state.append(float(self.prev_action[1]))  # acción angular previa
        return np.array(state, dtype=np.float32)

    # ====== Bucle de control dentro del execute_callback ======
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal with DRL")
        self.prev_action = [0.0, 0.0]

        # bucle a ~10 Hz
        while rclpy.ok():
            # cancelación
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancelled")
                self.cmd_pub.publish(Twist())
                goal_handle.canceled()
                return NavigateToPose.Result()

            # construir estado
            state = self.build_state()
            state_t = torch.from_numpy(state).unsqueeze(0).to(self.device)

            # inferencia
            with torch.no_grad():
                action = self.actor(state_t).cpu().numpy()[0]

            a_lin = action[0]
            a_ang = action[1]

            # desnormalización EXACTA del entorno
            linear = (a_lin + 1.0) / 2.0 * SPEED_LINEAR_MAX
            angular = a_ang * SPEED_ANGULAR_MAX

            twist = Twist()
            twist.linear.x = float(linear)
            twist.angular.z = float(angular)
            self.cmd_pub.publish(twist)

            self.prev_action = [float(a_lin), float(a_ang)]

            self.get_logger().info(
                f"dist={self.goal_distance:.3f}, lin={linear:.3f}, ang={angular:.3f}"
            )

            # llegada al goal
            if self.goal_distance < THREHSOLD_GOAL:
                self.get_logger().info("Goal reached by DRL")
                self.cmd_pub.publish(Twist())
                goal_handle.succeed()
                return NavigateToPose.Result()

            time.sleep(0.1)  # 10 Hz


def main(args=None):
    rclpy.init(args=args)
    node = DRLNavigator()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
