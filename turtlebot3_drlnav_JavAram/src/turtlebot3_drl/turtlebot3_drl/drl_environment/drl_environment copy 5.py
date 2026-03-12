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

# Automatically retrieved from Gazebo model configuration (40 by default).
NUM_SCAN_SAMPLES = 40
LINEAR = 0
ANGULAR = 1
MAX_GOAL_DISTANCE = math.sqrt(ARENA_LENGTH**2 + ARENA_WIDTH**2)

# Local map window size (cells) to include in state
LOCAL_MAP_SIZE = 40  # will produce 40x40 = 1600 map cells in state (adjust if needed)

class DRLEnvironment(Node):
    def __init__(self):
        super().__init__('drl_environment')
        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}")
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS

        self.scan_topic = TOPIC_SCAN
        self.velo_topic = TOPIC_VELO
        self.odom_topic = TOPIC_ODOM

        # Initialize variables and robot position
        self.goal_x, self.goal_y = 0.0, 0.0
        self.robot_x, self.robot_y = 0.0, 0.0
        self.robot_x_prev, self.robot_y_prev = 0.0, 0.0   # used for total_distance every 32 steps
        self.robot_x_odom_prev, self.robot_y_odom_prev = 0.0, 0.0  # used for path accumulation
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

        # Initialize LiDAR scan ranges from settings.py
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP

        self.difficulty_radius = 1
        self.local_step = 0
        self.time_sec = 0

        # Map / discovery tracking
        self.known_cells_set = set()        # set of (ix, iy) in world discrete coords
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_grid = None
        self.prev_map_grid = None
        self.visit_counts = None
        self.last_map_n_new_global = 0
        self.last_map_new_mask = None

        # Reward / discovery bookkeeping
        self.last_reward_pose = (self.robot_x, self.robot_y)
        self.path_since_last_reward = 0.0

        # Coverage stability counter
        self.coverage_stable_count = 0

        # Default parameters (adjustable)
        self.d_min = 0.15                 # net displacement threshold (m)
        self.min_new_cells_for_reward = 3 # minimum new cells to consider discovery
        self.min_cells_per_meter = 2.0    # minimum cells per meter ratio

        self.pose = {}

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(depth=1)
        qos_clock.reliability = ReliabilityPolicy.BEST_EFFORT

        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.velo_topic, qos)
        # subscribers
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub = self.create_subscription(Odometry, 'obstacle/odom', self.obstacle_odom_callback, qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, qos)
        # clients
        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')
        # servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def obstacle_odom_callback(self, msg):
        if 'obstacle' in msg.child_frame_id:
            robot_pos = msg.pose.pose.position
            obstacle_id = int(msg.child_frame_id[-1]) - 1
            diff_x = self.robot_x - robot_pos.x
            diff_y = self.robot_y - robot_pos.y
            self.obstacle_distances[obstacle_id] = math.sqrt(diff_y**2 + diff_x**2)
        else:
            print("ERROR: received odom was not from obstacle!")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

        # calculate traveled distance, every 32 steps, for logging (existing logic)
        if self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2)
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y

        # accumulate path length for discovery gating (every odom update)
        dx_odom = self.robot_x - getattr(self, 'robot_x_odom_prev', self.robot_x)
        dy_odom = self.robot_y - getattr(self, 'robot_y_odom_prev', self.robot_y)
        step_dist = math.sqrt(dx_odom*dx_odom + dy_odom*dy_odom)
        self.path_since_last_reward += step_dist
        self.robot_x_odom_prev = self.robot_x
        self.robot_y_odom_prev = self.robot_y

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

    
    # util: convertir índice de mapa (col,row) a índice mundo discreto (ix,iy)
    def _map_index_to_world_index(self, col, row, origin_x, origin_y, resolution):
        # center coordinate of the cell in world frame
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        ix = int(round(x / resolution))
        iy = int(round(y / resolution))
        return (ix, iy)

    # util: convert world coordinates to map indices (col, row) for current map
    def world_to_map(self, x, y):
        if self.map_resolution is None or self.map_origin_x is None or self.map_origin_y is None or self.map_grid is None:
            return None
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        # check bounds
        h, w = self.map_grid.shape
        if col < 0 or col >= w or row < 0 or row >= h:
            return None
        return (col, row)

    def map_callback(self, msg):
        h = int(msg.info.height)
        w = int(msg.info.width)
        resolution = float(msg.info.resolution)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)

        # detect changes in origin/resolution (relocalization) ------------------------------------------------
        origin_changed = False
        if getattr(self, 'map_origin_x', None) is not None and getattr(self, 'map_origin_y', None) is not None:
            dx_origin = origin_x - self.map_origin_x
            dy_origin = origin_y - self.map_origin_y
            origin_shift = math.sqrt(dx_origin*dx_origin + dy_origin*dy_origin)
            # threshold: e.g., 5 cells of previous resolution (if available) or 0.5 m fallback
            prev_res = getattr(self, 'map_resolution', resolution)
            relocal_threshold = max(5 * prev_res, 0.5)
            if origin_shift > relocal_threshold or abs(resolution - prev_res) > 1e-6:
                origin_changed = True
                self.get_logger().info(f"[MAP] Relocalization or resolution change detected: shift={origin_shift:.3f}m, prev_res={prev_res:.4f}, new_res={resolution:.4f}")

        # store resolution/origin (update early so helpers can use them)
        self.map_resolution = resolution
        self.map_origin_x = origin_x
        self.map_origin_y = origin_y

        # read and reshape data safely -----------------------------------------------------------------------
        try:
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        except Exception as e:
            data = np.array(msg.data, dtype=np.int8)
            if data.size == h * w:
                data = data.reshape((h, w))
            else:
                self.get_logger().error(f"OccupancyGrid shape mismatch: {e}")
                return

        # normalize to discrete values -1,0,1 ----------------------------------------------------------------
        norm_grid = np.empty_like(data, dtype=np.int8)
        norm_grid[data == -1] = -1
        norm_grid[data == 0] = 0
        norm_grid[data > 0] = 1
        # save previous map and set new map
        if getattr(self, 'map_grid', None) is None:
            self.prev_map_grid = norm_grid.copy()
            self.map_grid = norm_grid.copy()
        else:
            self.prev_map_grid = self.map_grid.copy()
            self.map_grid = norm_grid.copy()

        # if relocalization detected, clear or rebuild known_cells_set ------------------------------------------------
        if origin_changed:
            # opción segura: limpiar el histórico (evita inconsistencias)
            self.get_logger().info("[MAP] Clearing known_cells_set due to relocalization/resolution change.")
            self.known_cells_set.clear()
            # si prefieres reconstruir desde el nuevo mapa en lugar de limpiar, descomenta:
            # self.known_cells_set = set()
            # rows_all, cols_all = np.where(self.map_grid != -1)
            # for r, c in zip(rows_all, cols_all):
            #     key = self._map_index_to_world_index(c, r, origin_x, origin_y, resolution)
            #     self.known_cells_set.add(key)

        # compute known cells and coverage -------------------------------------------------------------------
        known_mask = (self.map_grid != -1)
        known_cells = int(known_mask.sum())
        total_cells = int(self.map_grid.size)
        self.coverage = known_cells / float(total_cells) if total_cells > 0 else 0.0

        # build current_set of world-discrete keys (only for known cells) -------------------------------------
        current_set = set()
        rows, cols = np.where(known_mask)
        for r, c in zip(rows, cols):
            key = self._map_index_to_world_index(c, r, origin_x, origin_y, resolution)
            current_set.add(key)

        # compute new cells relative to global known set -------------------------------------------------------
        new_cells_set = current_set - self.known_cells_set
        n_new_global = len(new_cells_set)

        # update global known set (union) --------------------------------------------------------------------
        if n_new_global > 0:
            # añadir solo las celdas actuales conocidas (evita mantener claves obsoletas)
            self.known_cells_set |= current_set

        # ensure visit_counts shape matches -------------------------------------------------------------------
        if getattr(self, 'visit_counts', None) is None or self.visit_counts.shape != self.map_grid.shape:
            self.visit_counts = np.zeros_like(self.map_grid, dtype=np.int16)

        # logging and coverage-stability (opcional) -----------------------------------------------------------
        self.get_logger().info(f"[MAP] Recibido {w}x{h}, known_cells={known_cells}, new_global={n_new_global}, coverage={self.coverage:.3f}")

        # opcional: mantener contador de estabilidad de coverage si tienes constantes definidas
        if getattr(self, 'COVERAGE_SUCCESS_THRESHOLD', None) is not None and self.coverage >= self.COVERAGE_SUCCESS_THRESHOLD:
            self.coverage_stable_count += 1
        else:
            self.coverage_stable_count = 0

        if getattr(self, 'COVERAGE_STABLE_STEPS', None) is not None and self.coverage_stable_count >= self.COVERAGE_STABLE_STEPS:
            self.succeed = SUCCESS

        # save for step usage ---------------------------------------------------------------------------------
        self.last_map_new_mask = new_cells_set
        self.last_map_n_new_global = n_new_global


    # Active everytime clock topic receives a msg and updates simulation time
    def clock_callback(self, msg):
        self.time_sec = msg.clock.sec
        if not self.reset_deadline:
            return
        self.clock_msgs_skipped += 1
        if self.clock_msgs_skipped <= 10:
            return
        episode_time = self.episode_timeout
        if ENABLE_DYNAMIC_GOALS:
            episode_time = numpy.clip(episode_time * self.difficulty_radius, 10, 50)
        self.episode_deadline = self.time_sec + episode_time
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

    # Stop the robot and reset the environment.
    def stop_reset_robot(self, success):
        self.cmd_vel_pub.publish(Twist())  # stop robot
        self.episode_deadline = Infinity
        self.done = True
        req = RingGoal.Request()
        req.robot_pose_x = self.robot_x
        req.robot_pose_y = self.robot_y
        req.radius = numpy.clip(self.difficulty_radius, 0.5, 4)
        if success:
            self.difficulty_radius *= 1.01
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('success service not available, waiting again...')
            self.task_succeed_client.call_async(req)
        else:
            self.difficulty_radius *= 0.99
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('fail service not available, waiting again...')
            self.task_fail_client.call_async(req)
        time.sleep(0.5)

    # Define the state with the current values of things.
    def get_state(self, action_linear_previous, action_angular_previous):
        """
        State composition:
         - LiDAR scan ranges (NUM_SCAN_SAMPLES floats)
         - previous actions (linear, angular)
        """
        state = copy.deepcopy(self.scan_ranges)  # range: [0,1]
        state.append(float(action_linear_previous))
        state.append(float(action_angular_previous))
        state.append(float(self.pose["x"]))
        state.append(float(self.pose["y"]))
        state.append(float(self.pose["yaw"]))
        state.append(float(self.map_grid.shape[0]))  # 505 -> H
        state.append(float(self.map_grid.shape[1]))  # 506 -> W
        state.extend(self.map_grid.flatten().tolist())

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
        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        # reset per-episode bookkeeping
        self.path_since_last_reward = 0.0
        self.last_reward_pose = (self.robot_x, self.robot_y)
        # clear known cells set per episode if desired (comment/uncomment)
        # self.known_cells_set.clear()
        return response

    # Active when other node calls step_comm service. Defines how an step is taken in the environment.
    def step_comm_callback(self, request, response):
        if len(request.action) == 0:  # If no action is provided, initialize episode
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

        # Prepare response state (use previous actions as stored in request)
        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR])

        # --- Discovery gating using map_callback result ---
        n_new = getattr(self, 'last_map_n_new_global', 0)

        # compute net displacement since last rewarded pose
        dx = self.robot_x - self.last_reward_pose[0]
        dy = self.robot_y - self.last_reward_pose[1]
        dist_since_last_reward = math.sqrt(dx*dx + dy*dy)

        # moved_enough: net displacement OR accumulated path length
        moved_enough = (dist_since_last_reward >= self.d_min) or (self.path_since_last_reward >= self.d_min)

        # cells per meter ratio (guard against zero)
        cells_per_meter = n_new / max(1e-6, dist_since_last_reward)

        # forward motion heuristic (allow small forward moves to count)
        MIN_FORWARD_V = 0.05
        MAX_ANGULAR_FOR_FORWARD = 0.6
        forward_motion = (action_linear > MIN_FORWARD_V) and (abs(action_angular) <= MAX_ANGULAR_FOR_FORWARD)

        # final condition to allow discovery reward
        allow_discovery_reward = moved_enough and (n_new >= self.min_new_cells_for_reward) and (cells_per_meter >= self.min_cells_per_meter or forward_motion)

        # if discovery reward is granted, reset path accumulator and last_reward_pose
        if allow_discovery_reward and n_new > 0:
            self.path_since_last_reward = 0.0
            self.last_reward_pose = (self.robot_x, self.robot_y)

        # convert previous action to m/s for reward extras (if previous action is normalized)
        prev_linear_raw = request.previous_action[LINEAR]
        if ENABLE_BACKWARD:
            prev_linear = prev_linear_raw * SPEED_LINEAR_MAX
        else:
            prev_linear = (prev_linear_raw + 1) / 2 * SPEED_LINEAR_MAX

        # Call reward function (wrapper accepts variable args)
        try:
            response.reward = float(rw.get_reward(self.succeed,
                                                  action_linear,
                                                  action_angular,
                                                  self.obstacle_distance,
                                                  n_new,
                                                  allow_discovery_reward,
                                                  dist_since_last_reward,
                                                  prev_linear))
        except Exception as e:
            # fallback: if reward fails, log and set small negative step penalty
            self.get_logger().error(f"Reward computation error: {e}")
            response.reward = -0.001

        # Diagnostic log for discovery gating
        self.get_logger().debug(f"[DISC] n_new={n_new} moved={moved_enough} dist={dist_since_last_reward:.3f} path={self.path_since_last_reward:.3f} ratio={cells_per_meter:.2f} forward={forward_motion} allow={allow_discovery_reward}")

        # Maintain response flags and distance traveled
        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0
        if self.done:
            response.distance_traveled = self.total_distance
            # Reset episode-level variables
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False
            self.reset_deadline = True
            # reset per-episode bookkeeping
            self.path_since_last_reward = 0.0
            self.last_reward_pose = (self.robot_x, self.robot_y)
            # clear known cells set if you want per-episode maps (comment/uncomment as desired)
            # self.known_cells_set.clear()
            self.last_map_n_new_global = 0

        # Periodic console print (every 200 steps)
        if self.local_step % 200 == 0:
            print(f"Rtot: {response.reward:<8.2f}\t", end='')
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
