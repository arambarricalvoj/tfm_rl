#!/usr/bin/env python3

import os
import time
import subprocess
import xml.etree.ElementTree as ET

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Twist

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from turtlebot3_msgs.srv import RingGoal
from ..common.settings import ARENA_LENGTH, ARENA_WIDTH


NO_GOAL_SPAWN_MARGIN = 0.3


class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        self.cb_group = ReentrantCallbackGroup()

        self.robot_name = "create3"

        # Goal entity (kept for compatibility but unused)
        self.entity_dir_path = (os.path.dirname(os.path.realpath(__file__))).replace(
            'turtlebot3_drl/lib/python3.10/site-packages/turtlebot3_drl/drl_gazebo',
            'turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_drl_world/goal_box')
        self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goal'

        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10, callback_group=self.cb_group
        )

        # Clients
        self.delete_entity_client = self.create_client(
            DeleteEntity, 'delete_entity', callback_group=self.cb_group
        )
        self.spawn_entity_client = self.create_client(
            SpawnEntity, 'spawn_entity', callback_group=self.cb_group
        )
        self.reset_simulation_client = self.create_client(
            Empty, 'reset_simulation', callback_group=self.cb_group
        )

        # Services
        self.task_succeed_server = self.create_service(
            RingGoal, 'task_succeed', self.task_succeed_callback,
            callback_group=self.cb_group
        )
        self.task_fail_server = self.create_service(
            RingGoal, 'task_fail', self.task_fail_callback,
            callback_group=self.cb_group
        )

        # Obstacles
        self.obstacle_coordinates = self.get_obstacle_coordinates()

        # Init world
        self.init_callback()

        # Start SLAM Toolbox
        self.start_slam_toolbox()

        # Primera vez: esperar mapa (el entorno DRL también espera su primer /map)
        self.get_logger().info("Waiting for initial /map from SLAM Toolbox...")
        self.wait_for_map()

    # -------------------------------------------------------------------------
    # INITIALIZATION
    # -------------------------------------------------------------------------
    def init_callback(self):
        self.reset_simulation()
        print("Init done")
        time.sleep(1)

    # -------------------------------------------------------------------------
    # EPISODE RESET LOGIC
    # -------------------------------------------------------------------------
    def task_succeed_callback(self, request, response):
        print("Episode success → resetting world + SLAM")
        self.reset_simulation()
        self.restart_slam_toolbox()
        return response

    def task_fail_callback(self, request, response):
        print("Episode fail → resetting world + SLAM")
        self.reset_simulation()
        self.restart_slam_toolbox()
        return response

    # -------------------------------------------------------------------------
    # SLAM TOOLBOX MANAGEMENT
    # -------------------------------------------------------------------------
    def start_slam_toolbox(self):
        print("Starting SLAM Toolbox...")
        self.slam_process = subprocess.Popen([
            "ros2", "launch", "slam_toolbox", "online_async_launch.py",
            "use_sim_time:=True",
            "params_file:=src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml"
        ])
        time.sleep(1.0)
        print("SLAM Toolbox started.")

    def restart_slam_toolbox(self):
        print("Killing SLAM Toolbox...")
        os.system("pkill -f slam_toolbox")
        time.sleep(0.5)

        print("Restarting SLAM Toolbox...")
        self.slam_process = subprocess.Popen([
            "ros2", "launch", "slam_toolbox", "online_async_launch.py",
            "use_sim_time:=True",
            "params_file:=src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml"
        ])

        print("Waiting for /map after restart...")
        self.wait_for_map()
        print("SLAM Toolbox restarted.")

    def wait_for_map(self):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        msg = None

        def cb(m):
            nonlocal msg
            msg = m

        sub = self.create_subscription(
            OccupancyGrid, "/map", cb, qos_map,
            callback_group=self.cb_group
        )

        timeout = time.time() + 20.0
        while rclpy.ok() and msg is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if msg is None:
            self.get_logger().warn("SLAM Toolbox did not publish /map in time")
        else:
            self.get_logger().info("SLAM Toolbox published /map.")

    # -------------------------------------------------------------------------
    # SIMULATION RESET
    # -------------------------------------------------------------------------
    def reset_simulation(self):
        self.move_robot(0.0, 0.0, duration=0.5)
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(req)

    def move_robot(self, linear_x=0.0, angular_z=0.0, duration=0.1):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        time.sleep(duration)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot stopped.")

    # -------------------------------------------------------------------------
    # ENTITY MANAGEMENT
    # -------------------------------------------------------------------------
    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.delete_entity_client.call_async(req)

    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = 0.0
        goal_pose.position.y = 0.0
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_entity_client.call_async(req)

    # -------------------------------------------------------------------------
    # OBSTACLE PARSING
    # -------------------------------------------------------------------------
    def get_obstacle_coordinates(self):
        tree = ET.parse(os.getenv('DRLNAV_BASE_PATH') + '/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/inner_walls/model.sdf')
        root = tree.getroot()
        obstacle_coordinates = []
        for wall in root.find('model').findall('link'):
            pose = wall.find('pose').text.split(" ")
            size = wall.find('collision').find('geometry').find('box').find('size').text.split()
            rotation = float(pose[-1])
            pose_x = float(pose[0])
            pose_y = float(pose[1])
            if rotation == 0:
                size_x = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
            else:
                size_x = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
            point_1 = [pose_x + size_x / 2, pose_y + size_y / 2]
            point_2 = [point_1[0], point_1[1] - size_y]
            point_3 = [point_1[0] - size_x, point_1[1] - size_y]
            point_4 = [point_1[0] - size_x, point_1[1]]
            wall_points = [point_1, point_2, point_3, point_4]
            obstacle_coordinates.append(wall_points)
        return obstacle_coordinates


def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)
    drl_gazebo.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()