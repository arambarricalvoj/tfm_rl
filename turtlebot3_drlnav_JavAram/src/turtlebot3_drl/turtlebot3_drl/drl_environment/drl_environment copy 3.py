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
import numpy as np
import sys
import copy
from math import inf as Infinity
import time

from geometry_msgs.msg import Pose, Twist
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

# Automatically retrieve from Gazebo model configuration (40 by default).
# Can be set manually if needed.
NUM_SCAN_SAMPLES = 40 #500 #util.get_scan_count() Set manually according to settings.py
LINEAR = 0
ANGULAR = 1
MAX_GOAL_DISTANCE = math.sqrt(ARENA_LENGTH**2 + ARENA_WIDTH**2)

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
        #self.goal_topic = 'goal_pose'

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

        # Initialize LiDAR scan ranges from settings.py
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP

        self.difficulty_radius = 1
        self.local_step = 0
        self.time_sec = 0

        """************************************************************
        ** Exploration / map bookkeeping
        ************************************************************"""
        # Map arrays (slam_toolbox OccupancyGrid: -1 unknown, 0 free, 100 occ)
        self.map_grid = None            # numpy array shape (height, width)
        self.prev_map_grid = None
        self.visit_counts = None        # same shape, int16
        self.coverage = 0.0
        self.coverage_stable_count = 0

        # Parameters for exploration reward and anti-spin (ajustables)
        self.d_min = 0.15               # m, mínimo desplazamiento para permitir reward por descubrimiento
        self.last_reward_pose = (self.robot_x, self.robot_y)
        self.alpha_spin = 0.02          # penalización por rad/s (si se aplica en entorno)
        self.revisit_threshold = 3      # visitas tras las cuales la recompensa por esa celda decae
        self.cells_discovered_episode = 0
        self.collisions_episode = 0

        # Coverage success threshold (ejemplo: 80%) y estabilidad
        self.COVERAGE_SUCCESS_THRESHOLD = 0.80
        self.COVERAGE_STABLE_STEPS = 5
        self.SUCCESS_BONUS = 20.0       # bonus que reward.py también puede aplicar; aquí lo dejamos para consistencia

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(depth=1)
        qos_clock.reliability = ReliabilityPolicy.BEST_EFFORT # To fit current clock publisher

        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.velo_topic, qos)
        # subscribers
        #self.goal_pose_sub = self.create_subscription(Pose, self.goal_topic, self.goal_pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub = self.create_subscription(Odometry, 'obstacle/odom', self.obstacle_odom_callback, qos)
        # map subscriber (slam_toolbox)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        # clients
        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')
        # servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        #self.goal_comm_server = self.create_service(Goal, 'goal_comm', self.goal_comm_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    # Active everytime goal_pose topic receives a msg and updates goal position
    def goal_pose_callback(self, msg):
        self.goal_x = msg.position.x
        self.goal_y = msg.position.y
        # keep compatibility but exploration ignores explicit goals
        print(f"new goal! x: {self.goal_x} y: {self.goal_y}")

    # Active when called goal_comm service
    def goal_comm_callback(self, request, response):
        response.new_goal = False
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

        # Note: goal_distance/angle are kept for compatibility but not used for exploration reward
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
                self.scan_ranges[i] = np.clip(float(msg.ranges[i]) / LIDAR_DISTANCE_CAP, 0, 1)
                if self.scan_ranges[i] < self.obstacle_distance:
                    self.obstacle_distance = self.scan_ranges[i]
        self.obstacle_distance *= LIDAR_DISTANCE_CAP

    # Map callback: update map arrays and coverage
    def map_callback(self, msg):
        h = msg.info.height
        w = msg.info.width
        try:
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        except Exception as e:
            # fallback if shape mismatch
            data = np.array(msg.data, dtype=np.int8)
            if data.size == h * w:
                data = data.reshape((h, w))
            else:
                print("ERROR: OccupancyGrid data shape mismatch:", e)
                return

        if self.map_grid is None:
            self.prev_map_grid = data.copy()
            self.map_grid = data.copy()
            self.visit_counts = np.zeros_like(data, dtype=np.int16)
        else:
            self.prev_map_grid = self.map_grid.copy()
            self.map_grid = data.copy()

        # update coverage metric
        known_mask = (self.map_grid != -1)
        known_cells = int(known_mask.sum())
        total_cells = int(self.map_grid.size)
        self.coverage = known_cells / float(total_cells) if total_cells > 0 else 0.0

        self.get_logger().info(f"[MAP] Recibido OccupancyGrid: {msg.info.width}x{msg.info.height}, coverage={self.coverage:.3f}")

        # update stable coverage counter
        if self.coverage >= self.COVERAGE_SUCCESS_THRESHOLD:
            self.coverage_stable_count += 1
        else:
            self.coverage_stable_count = 0

        # If coverage stable for enough steps, mark success
        if self.coverage_stable_count >= self.COVERAGE_STABLE_STEPS:
            self.succeed = SUCCESS
            # stop/reset will be handled in get_state

    # Active everytime clock topic receives a msg and updates simulation time
    def clock_callback(self, msg):
        self.time_sec = msg.clock.sec
        if not self.reset_deadline: # If reset deadline flag is set, reset episode deadline
            return
        self.clock_msgs_skipped += 1
        if self.clock_msgs_skipped <= 10: # Wait a few message for simulation to reset clock
            return
        episode_time = self.episode_timeout # Set episode time from settings.py
        if ENABLE_DYNAMIC_GOALS:
            episode_time = np.clip(episode_time * self.difficulty_radius, 10, 50) # Adapt episode time according to current difficulty radius
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
        req.radius = np.clip(self.difficulty_radius, 0.5, 4)
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
        time.sleep(0.5)

    # Define the state with the current values of things. Important function.
    def get_state(self, action_linear_previous, action_angular_previous):
        # Build state: LiDAR scans + previous actions (no goal-related entries for exploration)
        state = copy.deepcopy(self.scan_ranges)                                             # range: [ 0, 1]
        state.append(float(action_linear_previous))                                         # range: [-1, 1] or [0,1] depending normalization
        state.append(float(action_angular_previous))                                        # range: [-1, 1]
        self.local_step += 1

        if self.local_step <= 30: # Grace period to wait for simulation reset
            return state

        # Check terminal conditions (collision, timeout, tumble, coverage success)
        # Collision
        if self.obstacle_distance < THRESHOLD_COLLISION: # obstacle_distance is the minmum distance from LiDAR
            dynamic_collision = False
            for obstacle_distance in self.obstacle_distances: # iterate dynamic obstacle distances to robot.
                if obstacle_distance < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05):
                    dynamic_collision = True
            if dynamic_collision:
                self.succeed = COLLISION_OBSTACLE
            else:
                self.succeed = COLLISION_WALL
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.succeed = TIMEOUT
        # Tumble. This activates if the robot falls over.
        elif self.robot_tilt > 0.06 or self.robot_tilt < -0.06:
            self.succeed = TUMBLE
        # Coverage success handled in map_callback (sets self.succeed = SUCCESS when stable)

        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)
        return state

    # Intialize the episode
    def initalize_episode(self, response):
        # For compatibility with reward_initalize, pass None (exploration doesn't need initial goal distance)
        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        rw.reward_initalize(None)
        # reset exploration bookkeeping
        self.cells_discovered_episode = 0
        self.collisions_episode = 0
        self.coverage_stable_count = 0
        self.last_reward_pose = (self.robot_x, self.robot_y)
        return response

    # Utility: count new cells between prev_map_grid and map_grid
    def count_new_cells(self):
        if self.prev_map_grid is None or self.map_grid is None:
            return 0, None

        # Obtener tamaños
        h1, w1 = self.prev_map_grid.shape
        h2, w2 = self.map_grid.shape

        # Tamaño común
        h = min(h1, h2)
        w = min(w1, w2)

        prev = self.prev_map_grid[:h, :w]
        new  = self.map_grid[:h, :w]

        new_mask = np.logical_and(prev == -1, new != -1)
        return int(new_mask.sum()), new_mask


    # Active when other node calls step_comm service. Defines how an step is taken in the environment.
    def step_comm_callback(self, request, response):
        if len(request.action) == 0: # If no action is provided, initialize episode
            return self.initalize_episode(response)

        if ENABLE_MOTOR_NOISE:
            request.action[LINEAR] += np.clip(np.random.normal(0, 0.05), -0.1, 0.1)
            request.action[ANGULAR] += np.clip(np.random.normal(0, 0.05), -0.1, 0.1)

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

        # Prepare response state
        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR]) # Get state with the actions using get_state function

        # Count newly discovered cells (cheap binary comparison)
        n_new, new_mask = self.count_new_cells()

        # Condition: only grant discovery reward if robot moved at least d_min since last reward
        dx = self.robot_x - self.last_reward_pose[0]
        dy = self.robot_y - self.last_reward_pose[1]
        dist_since_last_reward = math.sqrt(dx*dx + dy*dy)
        allow_discovery_reward = dist_since_last_reward >= self.d_min

        # Compute base reward via reward module (wrapper supports both old and new signatures)
        # New signature for exploration: (succeed, action_linear, action_angular, min_obstacle_dist, n_new, allow_discovery_reward, dist_since_last_reward)
        try:
            response.reward = float(rw.get_reward(self.succeed,
                                                  action_linear,
                                                  action_angular,
                                                  self.obstacle_distance,
                                                  n_new,
                                                  allow_discovery_reward,
                                                  dist_since_last_reward))
        except Exception:
            # Fallback to old signature for compatibility
            response.reward = float(rw.get_reward(self.succeed,
                                                  action_linear,
                                                  action_angular,
                                                  self.goal_distance,
                                                  self.goal_angle,
                                                  self.obstacle_distance))

        # Add local anti-spin penalty if desired (keeps behaviour consistent even if reward.py doesn't)
        r_spin = - self.alpha_spin * abs(action_angular)
        response.reward += r_spin

        # If discovery allowed and there are new cells, update visit counts and last_reward_pose
        if allow_discovery_reward and n_new > 0 and new_mask is not None:
            self.cells_discovered_episode += n_new
            try:
                # increment visit counts for newly known cells
                self.visit_counts[new_mask] += 1
            except Exception:
                pass
            self.last_reward_pose = (self.robot_x, self.robot_y)

        # Collision accounting
        if self.succeed in (COLLISION_WALL, COLLISION_OBSTACLE):
            self.collisions_episode += 1

        # If success (coverage reached), add success bonus to reward (also reward.py may add one)
        if self.succeed == SUCCESS:
            response.reward += self.SUCCESS_BONUS

        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0 # Will be updated at the end of episode

        if self.done:
            response.distance_traveled = self.total_distance
            # Log episode metrics
            print(f"Episode finished. Discovered cells: {self.cells_discovered_episode}, Collisions: {self.collisions_episode}, Coverage: {self.coverage:.3f}")
            # Reset variables for next episode
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False
            self.reset_deadline = True
            self.cells_discovered_episode = 0
            self.collisions_episode = 0
            self.coverage_stable_count = 0
            self.last_reward_pose = (self.robot_x, self.robot_y)

        if self.local_step % 200 == 0: # Log every 200 steps, print useful info in console
            print(f"Rtot: {response.reward:<8.2f} MinD: {self.obstacle_distance:<8.2f} Alin: {request.action[LINEAR]:<7.1f} Aturn: {request.action[ANGULAR]:<7.1f} Coverage: {self.coverage:.3f}")
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
