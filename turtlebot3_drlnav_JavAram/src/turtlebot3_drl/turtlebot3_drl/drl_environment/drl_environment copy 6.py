#
#!/usr/bin/env python3
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert, Tomas

import math
import numpy
import sys
import copy
from math import inf as Infinity
import time

from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import DrlStep, Goal, RingGoal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy

from . import reward as rw
from ..common import utilities as util
from ..common.settings import ENABLE_BACKWARD, EPISODE_TIMEOUT_SECONDS, ENABLE_MOTOR_NOISE, UNKNOWN, SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE, \
                                TOPIC_SCAN, TOPIC_VELO, TOPIC_ODOM, ARENA_LENGTH, ARENA_WIDTH, MAX_NUMBER_OBSTACLES, OBSTACLE_RADIUS, LIDAR_DISTANCE_CAP, \
                                    SPEED_LINEAR_MAX, SPEED_ANGULAR_MAX, THRESHOLD_COLLISION, THREHSOLD_GOAL, ENABLE_DYNAMIC_GOALS

from nav_msgs.msg import OccupancyGrid
import numpy as np

# Automatically retrievew from Gazebo model configuration (40 by default).
# Can be set manually if needed.
NUM_SCAN_SAMPLES = 500
LINEAR = 0
ANGULAR = 1
MAX_GOAL_DISTANCE = math.sqrt(ARENA_LENGTH**2 + ARENA_WIDTH**2)

GLOBAL_RES = 0.05      # m por celda global (fija)
LOCAL_CELL_SIZE = 0.5  # metros por celda en la grid local
LOCAL_GRID_N = 6       # 6x6
LOCAL_GRID_FLAT = LOCAL_GRID_N * LOCAL_GRID_N  # 36
POSE_SCALE = 10.0   # normaliza x,y (metros)
# state_size esperado: NUM_SCAN_SAMPLES + 36 + 4 + 2 + 1 = 543

class DRLEnvironment(Node):
    def __init__(self):
        super().__init__('drl_environment')
        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}")
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS

        self.scan_topic = TOPIC_SCAN #Define scan topic from settings.py
        self.velo_topic = TOPIC_VELO #Define velocity command topic from settings.py
        self.odom_topic = TOPIC_ODOM #Define odometry topic from settings.py
        self.goal_topic = 'goal_pose'

        # Initialize variables, goal and robot position
        self.goal_x, self.goal_y = 0.0, 0.0
        self.robot_x, self.robot_y = 0.0, 0.0
        self.robot_x_prev, self.robot_y_prev = 0.0, 0.0
        self.robot_heading = 0.0
        self.total_distance = 0.0
        self.robot_tilt = 0.0

        # Initialize episode variables
        self.done = False
        self.succeed = UNKNOWN
        self.episode_deadline = Infinity
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

        # Initialize obstacle distances (as infinite)
        self.obstacle_distances = [Infinity] * MAX_NUMBER_OBSTACLES

        # Initialize goal variables
        self.new_goal = False
        self.goal_angle = 0.0
        self.goal_distance = MAX_GOAL_DISTANCE
        self.initial_distance_to_goal = MAX_GOAL_DISTANCE

        # Initialize LiDAR scan ranges from settings.py
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP

        self.difficulty_radius = 1
        self.local_step = 0
        self.time_sec = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(depth=1)
        qos_clock.reliability = ReliabilityPolicy.BEST_EFFORT # To fit current clock publisher

        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.velo_topic, qos)
        # subscribers
        self.goal_pose_sub = self.create_subscription(Pose, self.goal_topic, self.goal_pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub = self.create_subscription(Odometry, 'obstacle/odom', self.obstacle_odom_callback, qos)
        # clients
        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')
        # servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(Goal, 'goal_comm', self.goal_comm_callback)

        # en __init__ del nodo 
        self.map_grid = None 
        self.prev_map_grid = None 
        self.map_resolution = None 
        self.map_origin_x = None 
        self.map_origin_y = None 
        self.local_grid_6x6 = None 
        self.known_cells_count = 0 
        self.known_cells_count_prev = 0
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0} 
        self.subscription = self.create_subscription( OccupancyGrid, '/map', self.map_callback, 10 ) 
        self.pose_sub = self.create_subscription( PoseWithCovarianceStamped, '/pose', self.pose_callback, qos )
        self.coverage = 0.0
    
    
    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""     
    def compute_local_grid_6x6(self, robot_x, robot_y, robot_yaw):
        if self.map_grid is None:
            return np.zeros((LOCAL_GRID_N, LOCAL_GRID_N), dtype=np.uint8)

        known_mask = (self.map_grid != -1).astype(np.uint8)
        h, w = self.map_grid.shape
        res = self.map_resolution
        ox = self.map_origin_x
        oy = self.map_origin_y

        offsets = (np.arange(LOCAL_GRID_N) - (LOCAL_GRID_N - 1) / 2.0) * LOCAL_CELL_SIZE
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)

        local_grid = np.zeros((LOCAL_GRID_N, LOCAL_GRID_N), dtype=np.uint8)

        for ri, dy in enumerate(offsets):
            for cj, dx in enumerate(offsets):
                wx = robot_x + (dx * cos_y - dy * sin_y)
                wy = robot_y + (dx * sin_y + dy * cos_y)

                # usar floor para índices (maneja coordenadas negativas)
                col = int(math.floor((wx - ox) / res))
                row = int(math.floor((wy - oy) / res))

                if row < 0 or row >= h or col < 0 or col >= w:
                    local_grid[ri, cj] = 0
                    continue

                r0 = max(0, row - 1)
                r1 = min(h - 1, row + 1)
                c0 = max(0, col - 1)
                c1 = min(w - 1, col + 1)

                block = known_mask[r0:r1+1, c0:c1+1]
                frac_known = float(block.sum()) / float(block.size)
                local_grid[ri, cj] = 1 if frac_known >= 0.5 else 0

        return local_grid

    # map_callback mínimo y robusto
    def map_callback(self, msg):
        h = int(msg.info.height)
        w = int(msg.info.width)
        resolution = float(msg.info.resolution)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)

        # actualizar metadatos
        self.map_resolution = resolution
        self.map_origin_x = origin_x
        self.map_origin_y = origin_y

        # leer y reshapar
        try:
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        except Exception:
            data = np.array(msg.data, dtype=np.int8)
            if data.size == h * w:
                data = data.reshape((h, w))
            else:
                self.get_logger().error("[MAP] OccupancyGrid shape mismatch")
                return

        # normalizar a -1,0,1
        norm_grid = np.empty_like(data, dtype=np.int8)
        norm_grid.fill(-1)
        norm_grid[data == 0] = 0
        norm_grid[data > 0] = 1

        # actualizar prev/current
        if self.map_grid is None:
            self.prev_map_grid = norm_grid.copy()
            self.map_grid = norm_grid.copy()
        else:
            self.prev_map_grid = self.map_grid.copy()
            self.map_grid = norm_grid.copy()

        # contar celdas conocidas (cantidad absoluta)
        known_cells_count = int((self.map_grid != -1).sum())
        self.known_cells_count_prev = self.known_cells_count
        self.known_cells_count = known_cells_count
        
        # calcular coverage como fracción de celdas conocidas sobre el mapa actual 
        total_cells = int(self.map_grid.size) if self.map_grid is not None else 0 
        if total_cells > 0: 
            self.coverage = float(known_cells_count) / float(total_cells) 
        else: 
            self.coverage = 0.0

        # construir local grid 6x6 centrada en la pose actual
        rx = float(self.pose.get('x', 0.0))
        ry = float(self.pose.get('y', 0.0))
        ryaw = float(self.pose.get('yaw', 0.0))
        self.local_grid_6x6 = self.compute_local_grid_6x6(rx, ry, ryaw)

        # logging mínimo (evitar spam)
        self.get_logger().info(f"[MAP] recibido {w}x{h}, known_cells={known_cells_count}, coverage={self.coverage}")


    """
    # índice -> centro mundo
    def map_index_to_world_center(col, row, origin_x, origin_y, resolution):
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        return x, y

    # mundo -> índice (usar floor para seguridad)
    def world_to_map_index(x, y, origin_x, origin_y, resolution):
        col = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        return col, row

    # mundo -> clave global
    def world_to_global_key(x, y, GLOBAL_RES):
        ix = int(round(x / GLOBAL_RES))
        iy = int(round(y / GLOBAL_RES))
        return ix, iy
    """

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg):
        """
        Actualiza self.pose = {'x': float, 'y': float, 'yaw': float}.
        Evita logging en cada callback; registra solo si hay cambios significativos.
        """
        # asegurar diccionario
        if not hasattr(self, 'pose') or self.pose is None:
            self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # extraer posición
        try:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
        except Exception:
            return  # mensaje mal formado

        # extraer yaw del cuaternión
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # actualizar solo si hay cambio apreciable (evita logs y trabajo innecesario)
        dx = abs(x - self.pose.get('x', 0.0))
        dy = abs(y - self.pose.get('y', 0.0))
        dyaw = abs((yaw - self.pose.get('yaw', 0.0) + math.pi) % (2*math.pi) - math.pi)

        self.pose['x'] = x
        self.pose['y'] = y
        self.pose['yaw'] = yaw

        # log solo si cambio significativo (ej.: > 1 cm o > 0.01 rad)
        if dx > 0.01 or dy > 0.01 or dyaw > 0.01:
            self.get_logger().info(f"Pose actualizada -> x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f} rad")
    
    # Active everytime goal_pose topic receives a msg and updates goal position
    def goal_pose_callback(self, msg):
        self.goal_x = msg.position.x
        self.goal_y = msg.position.y
        self.new_goal = True
        print(f"new goal! x: {self.goal_x} y: {self.goal_y}")

    # Active when called goal_comm service
    def goal_comm_callback(self, request, response):
        response.new_goal = self.new_goal
        return response
    
    # Active everytime obstacle odom topic receives a msg and updates obstacle positions
    def obstacle_odom_callback(self, msg):
        if 'obstacle' in msg.child_frame_id:
            robot_pos = msg.pose.pose.position
            obstacle_id = int(msg.child_frame_id[-1]) - 1
            diff_x = self.robot_x - robot_pos.x
            diff_y = self.robot_y - robot_pos.y
            self.obstacle_distances[obstacle_id] = math.sqrt(diff_y**2 + diff_x**2)
        else:
            print("ERROR: received odom was not from obstacle!")
    
    # Active everytime odom topic receives a msg and updates robot position using odom data
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

        # calculate traveled distance, every 32 steps, for logging
        if self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2)
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y
        # calculate distance and angle to goal
        diff_y = self.goal_y - self.robot_y
        diff_x = self.goal_x - self.robot_x
        distance_to_goal = math.sqrt(diff_x**2 + diff_y**2)
        heading_to_goal = math.atan2(diff_y, diff_x)
        goal_angle = heading_to_goal - self.robot_heading

        # Normalize goal angle to [-pi, pi]
        while goal_angle > math.pi:
            goal_angle -= 2 * math.pi
        while goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = distance_to_goal
        self.goal_angle = goal_angle

    # Active everytime scan topic receives a msg and save the reads in scan_ranges normalized using LIDAR_DISTANCE_CAP
    def scan_callback(self, msg):
        if len(msg.ranges) != NUM_SCAN_SAMPLES:
            print(f"more or less scans than expected! check model.sdf, got: {len(msg.ranges)}, expected: {NUM_SCAN_SAMPLES}")
        # normalize laser values
        self.obstacle_distance = 1
        for i in range(NUM_SCAN_SAMPLES):
                self.scan_ranges[i] = numpy.clip(float(msg.ranges[i]) / LIDAR_DISTANCE_CAP, 0, 1)
                if self.scan_ranges[i] < self.obstacle_distance: 
                    self.obstacle_distance = self.scan_ranges[i] 
        self.obstacle_distance *= LIDAR_DISTANCE_CAP
        #print("SCAN OUT: ",self.scan_ranges[:10])
        #print("MSG OUT: ",msg.ranges[:10])
        #print("OBSTACLE DISTANCE: ", self.obstacle_distance)
	
    # Active everytime clock topic receives a msg and updates simulation time
    def clock_callback(self, msg):
        self.time_sec = msg.clock.sec
        #print("TIME SEC: ", self.time_sec)
        if not self.reset_deadline: # If reset deadline flag is set, reset episode deadline
            return
        self.clock_msgs_skipped += 1
        if self.clock_msgs_skipped <= 10: # Wait a few message for simulation to reset clock
            return
        episode_time = self.episode_timeout # Set episode time from settings.py
        if ENABLE_DYNAMIC_GOALS:
            episode_time = numpy.clip(episode_time * self.difficulty_radius, 10, 50) # Adapt episode time according to current difficulty radius
        self.episode_deadline = self.time_sec + episode_time
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

    # Stop the robot and reset the environment. Not sure if this is working properly.
    def stop_reset_robot(self, success):
        self.cmd_vel_pub.publish(Twist()) # stop robot
        self.episode_deadline = Infinity
        self.done = True
        req = RingGoal.Request() # Prepare service request to send to task succeed/fail service
        req.robot_pose_x = self.robot_x
        req.robot_pose_y = self.robot_y
        req.radius = numpy.clip(self.difficulty_radius, 0.5, 4)
        if success: # The sucess is defined in get_state function
            self.difficulty_radius *= 1.01
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('success service not available, waiting again...')
            self.task_succeed_client.call_async(req)
        else:
            self.difficulty_radius *= 0.99
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('fail service not available, waiting again...')
            self.task_fail_client.call_async(req)
        self.get_logger().info('Entorno reiniciado.')
        time.sleep(0.5)
    # Define the state with the current values of things. Important function.

    def get_state(self, action_linear_previous, action_angular_previous):
        """
        Construye y devuelve un vector 1D np.float32 con el orden fijo:
        [ laser (NUM_SCAN_SAMPLES),
            local_grid_flat (36, row-major, valores en [0,1]),
            pose_norm (px_norm, py_norm, sin(yaw), cos(yaw)),
            prev_actions (a_lin_prev, a_ang_prev),
            coverage_frac (0..1)
        ]
        Requisitos previos: self.scan_ranges (len NUM_SCAN_SAMPLES), self.local_grid_6x6 (6x6),
        self.pose {'x','y','yaw'}, self.map_grid (opcional para coverage).
        """
        # --- 1) LIDAR (asegurar tamaño y tipo) ---
        lidar_arr = np.asarray(self.scan_ranges, dtype=np.float32)
        if lidar_arr.size != NUM_SCAN_SAMPLES:
            # pad with zeros or trim to ensure fixed length
            tmp = np.zeros(NUM_SCAN_SAMPLES, dtype=np.float32)
            n = min(lidar_arr.size, NUM_SCAN_SAMPLES)
            tmp[:n] = lidar_arr[:n]
            lidar_arr = tmp

        # --- 2) local grid 6x6 (flatten) ---
        # Esperamos valores en [0,1]. Si la grid es binaria uint8, la convertimos a float.
        if getattr(self, 'local_grid_6x6', None) is None:
            local_flat = np.zeros(LOCAL_GRID_FLAT, dtype=np.float32)
        else:
            lg = np.asarray(self.local_grid_6x6, dtype=np.float32)
            # Si la grid contiene 0/1 o fracciones, ya está en [0,1]; si no, clip
            lg = np.clip(lg, 0.0, 1.0)
            if lg.size != LOCAL_GRID_FLAT:
                # fallback seguro: reshape/pad/trim
                tmp = np.zeros(LOCAL_GRID_FLAT, dtype=np.float32)
                flat = lg.ravel()
                n = min(flat.size, LOCAL_GRID_FLAT)
                tmp[:n] = flat[:n]
                local_flat = tmp
            else:
                local_flat = lg.ravel().astype(np.float32)

        # --- 3) pose normalizada ---
        px = float(self.pose.get('x', 0.0))
        py = float(self.pose.get('y', 0.0))
        yaw = float(self.pose.get('yaw', 0.0))
        px_norm = float(np.clip(px / POSE_SCALE, -1.0, 1.0))
        py_norm = float(np.clip(py / POSE_SCALE, -1.0, 1.0))
        sy = float(math.sin(yaw))
        cy = float(math.cos(yaw))
        pose_vec = np.array([px_norm, py_norm, sy, cy], dtype=np.float32)

        # --- 4) acciones previas (ya en [-1,1]) ---
        a_lin = float(action_linear_previous)
        a_ang = float(action_angular_previous)
        actions_vec = np.array([a_lin, a_ang], dtype=np.float32)

        # --- 5) coverage (fracción 0..1) ---
        if getattr(self, 'map_grid', None) is not None:
            total = float(self.map_grid.size)
            known = float((self.map_grid != -1).sum())
            coverage_frac = known / total if total > 0 else 0.0
        else:
            coverage_frac = float(getattr(self, 'coverage', 0.0))
        coverage_vec = np.array([np.clip(coverage_frac, 0.0, 1.0)], dtype=np.float32)

        # --- 6) concatenar en orden fijo y devolver ---
        state_vec = np.concatenate([
            lidar_arr,
            local_flat,
            pose_vec,
            actions_vec,
            coverage_vec
        ]).astype(np.float32)

        # Sanitizar: convertir NaN/Inf a 0.0 y asegurar floats de Python
        # (evita que ROS2 rechace la secuencia)
        state = np.nan_to_num(state_vec, nan=0.0, posinf=1e6, neginf=-1e6).tolist()

        self.local_step += 1

        # early grace period
        if self.local_step <= 30:
            return state

        # Collision detection and episode termination checks
        if self.coverage >= 0.8:
            self.succeed = SUCCESS
        elif self.obstacle_distance < THRESHOLD_COLLISION:
            dynamic_collision = False
            for obstacle_distance in self.obstacle_distances:
                if obstacle_distance < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05):
                    dynamic_collision = True
            if dynamic_collision:
                self.succeed = COLLISION_OBSTACLE
            else:
                self.succeed = COLLISION_WALL
        elif self.time_sec >= self.episode_deadline:
            self.succeed = TIMEOUT
        elif self.robot_tilt > 0.06 or self.robot_tilt < -0.06:
            self.succeed = TUMBLE

        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)

        return state

    # Intialize the episode
    def initalize_episode(self, response):
        #self.initial_distance_to_goal = self.goal_distance
        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        rw.reward_initialize(self.initial_distance_to_goal)
        return response
    
    # Active when other node calls step_comm service. Defines how an step is taken in the environment.
    def step_comm_callback(self, request, response):
        # Debugging time outputs
        #ahora_ROS = self.get_clock().now()      # Tiempo ROS2 (builtin_interfaces/Time)
        #ahora_time =  time.time()            # Tiempo sistema operativo (float, segundos)
        #print(f"Tiempo ROS2 ACTION: {ahora_ROS}  Tiempo SO ACTION: {ahora_time}")    
        if len(request.action) == 0: # If no action is provided, initialize episode
            return self.initalize_episode(response)

        if ENABLE_MOTOR_NOISE:
            request.action[LINEAR] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)
            request.action[ANGULAR] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)

        # Un-normalize actions
        if ENABLE_BACKWARD:
            action_linear = request.action[LINEAR] * SPEED_LINEAR_MAX
        else:
            action_linear = (request.action[LINEAR] + 1) / 2 * SPEED_LINEAR_MAX
        action_angular = request.action[ANGULAR] * SPEED_ANGULAR_MAX

        # Publish action cmd to move the robot following the action provided 
        twist = Twist()
        twist.linear.x = action_linear
        twist.angular.z = action_angular
        self.cmd_vel_pub.publish(twist)

        # Prepare repsonse to send back to the caller
        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR]) # Get state with the actions using get_state function
        # Get reward using the reward function defined in reward.py
        extras = {'revisit_ratio': 0.0, 'spin_metric': 0.0}  # rellena si tienes esas métricas
        new_cells = max(0, self.known_cells_count - self.known_cells_count_prev)

        response.reward = float(rw.get_reward_explore(
            self.succeed,
            action_linear,
            action_angular,
            self.obstacle_distance,
            new_cells,
            True,                # allow_discovery_reward
            0.0,                 # dist_since_last_reward (opcional)
            extras               # extras (opcional)
        ))

        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0 # Will be updated at the end of episode
        if self.done:
            response.distance_traveled = self.total_distance
            # Reset variables
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False
            self.reset_deadline = True
        if self.local_step % 200 == 0: # Log every 200 steps, print useful info in console
            print(f"Rtot: {response.reward:<8.2f}", end='')
            print(f"MinD: {self.obstacle_distance:<8.2f}Alin: {request.action[LINEAR]:<7.1f}Aturn: {request.action[ANGULAR]:<7.1f}")
        return response

def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    if len(args) == 0:
        drl_environment = DRLEnvironment()
    else:
        rclpy.shutdown()
        quit("ERROR: wrong number of arguments!")
    rclpy.spin(drl_environment)
    drl_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()