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
import numpy as np
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

from .map_processing_utils import MapProcessor

from . import reward as rw
from ..common import utilities as util
from ..common.settings import ENABLE_BACKWARD, EPISODE_TIMEOUT_SECONDS, ENABLE_MOTOR_NOISE, UNKNOWN, SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE, \
                                TOPIC_SCAN, TOPIC_VELO, TOPIC_ODOM, ARENA_LENGTH, ARENA_WIDTH, MAX_NUMBER_OBSTACLES, OBSTACLE_RADIUS, LIDAR_DISTANCE_CAP, \
                                    SPEED_LINEAR_MAX, SPEED_ANGULAR_MAX, THRESHOLD_COLLISION, THREHSOLD_GOAL, ENABLE_DYNAMIC_GOALS

# Automatically retrievew from Gazebo model configuration (40 by default).
# Can be set manually if needed.
NUM_SCAN_SAMPLES = 500 #500 #util.get_scan_count() Set manually according to settings.py
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

        self.map_sub = self.create_subscription( OccupancyGrid, '/map', self.map_callback, 10 ) 
        self.processor = MapProcessor()
        self.processor.enable_lem = True
        self.processor.enable_global_reduced_map = True
        self.processor.lem_scale = 3.5
        
        # GUI del mapa del paquete map_processor
        try:
            self.processor.start_plot()
        except Exception as e:
            self.get_logger().warn(f'No se pudo iniciar plot: {e}')
        
        self.exploration = {'current': 0.0, 'previous': 0.0}

        self.map_known_percent_prev = 0.0
        self.map_known_percent_curr = 0.0
        self.pose_sub = self.create_subscription( PoseWithCovarianceStamped, '/pose', self.pose_callback, qos )
        # clients
        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')
        # servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(Goal, 'goal_comm', self.goal_comm_callback)

        self.map_resolution = 0.0
        self.map_origin_x = None
        self.map_origin_y = None
        self.prev_map_grid = None
        self.map_grid = None
        self.known_cells_count_prev = 0
        self.known_cells_count = 0
        self.coverage = 0.0
        self.cov_previous = 0.0
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.prev_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.diff_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        #self.steps_no_move = 0   # opcional, para penalización acumulativa

        self.steps = {'progress': 0, 'since_last_progress': 0, 'total': 0}

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def map_callback(self, msg: OccupancyGrid):
        # actualizar el objeto MapProcessor con el OccupancyGrid recibido
        try:
            self.current_map_msg = msg
            self.processor.update_from_occupancy_grid(msg)
        except Exception as e:
            self.get_logger().error(f'Error actualizando mapa: {e}')
            return
        
        self.exploration['previous'] = self.exploration['current']
        self.exploration['current'] = (100 - self.processor.unknown_percent_gem) / 100.0

        self.get_logger().info(
            f'Map updated: known%={100 - self.processor.unknown_percent_gem:.1f} '
        )

        # acceder a métricas ya calculadas y loguearlas o publicarlas
        #free = self.processor.free_percent
        #occ = self.processor.occupied_percent
        #known_pct = 100 - self.processor.unknown_percent
        #coverage = self.processor.free_ratio_known 
        #bbox = self.processor.explored_bbox
        #w = self.processor.width if self.processor.width is not None else 'N/A'
        #h = self.processor.height if self.processor.height is not None else 'N/A'

        

        """self.get_logger().info(
            f'Map updated: known%={known_pct:.1f} '
            f'bbox={bbox} '
            f'H={h} W={w}'
        )"""
    
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
            self.processor.set_robot_pose_from_pose_msg(msg)
            self.processor.update_plot()
        except Exception:
            return  # mensaje mal formado

        # extraer yaw del cuaternión
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.prev_pose['x'] = self.pose['x']
        self.prev_pose['y'] = self.pose['y']
        self.prev_pose['yaw'] = self.pose['yaw']
        
        self.pose['x'] = x
        self.pose['y'] = y
        self.pose['yaw'] = yaw

        # actualizar solo si hay cambio apreciable (evita logs y trabajo innecesario)
        dx = self.pose['x'] - self.prev_pose['x']
        dy = self.pose['y'] - self.prev_pose['y']
        dyaw = abs((self.pose['yaw'] - self.prev_pose['yaw'] + math.pi) % (2*math.pi) - math.pi)
        self.diff_pose['x'] = dx
        self.diff_pose['y'] = dy
        self.diff_pose['yaw'] = dyaw
      
        # log solo si cambio significativo (ej.: > 1 cm o > 0.01 rad)
        #if dx > 0.01 or dy > 0.01 or dyaw > 0.01:
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
        #distance_to_goal = math.sqrt(diff_x**2 + diff_y**2)
        #heading_to_goal = math.atan2(diff_y, diff_x)
        #goal_angle = heading_to_goal - self.robot_heading

        # Normalize goal angle to [-pi, pi]
        """while goal_angle > math.pi:
            goal_angle -= 2 * math.pi
        while goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = distance_to_goal
        self.goal_angle = goal_angle"""

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
        time.sleep(0.5)
    # Define the state with the current values of things. Important function.
    def get_state(self, action_linear_previous, action_angular_previous):
        import numpy as np, copy

        # base: copia de scan_ranges (lista de floats)
        state = list(copy.deepcopy(self.scan_ranges))  # range: [0,1], longitud NUM_SCAN_SAMPLES

        # maps
        lem = self.processor.lem_map
        gem = self.processor.gem_map
        if lem is None or gem is None:
            maps_vec = np.zeros(24*24*2, dtype=np.float32)
        else:
            # flatten y concatenar
            lem_f = lem.flatten().astype(np.float32)
            gem_f = gem.flatten().astype(np.float32)
            # normalizar (muy recomendable)
            lem_f /= 255.0
            gem_f /= 255.0
            maps_vec = np.concatenate([lem_f, gem_f], axis=0)
        state.extend(maps_vec)

        # acciones previas y yaw (escalas ya en [-1,1] y yaw en rad)
        state.append(float(action_linear_previous))
        state.append(float(action_angular_previous))

        state.append(np.sin(float(self.pose['yaw'])))
        state.append(np.cos(float(self.pose['yaw'])))

        state = [float(x) for x in state]

        logger = self.get_logger()
        for idx, v in enumerate(state):
            # Tipo incorrecto
            if not isinstance(v, float):
                logger.info(f"[STATE ERROR] idx={idx} tipo={type(v)} valor={v}")

            # NaN
            if isinstance(v, float) and (v != v):  # forma sin math.isnan
                logger.info(f"[STATE ERROR] idx={idx} valor=NaN")

            # Inf
            if isinstance(v, float) and (v == float('inf') or v == float('-inf')):
                logger.info(f"[STATE ERROR] idx={idx} valor=INF")



        #state.append(float(self.exploration['current']))
        #state.append(float(self.obstacle_distance/LIDAR_DISTANCE_CAP))


        self.local_step += 1
        if self.local_step <= 30: # Grace period to wait for simulation reset
            return state
        # Success
        if (100 - self.processor.unknown_percent) > 95.0:
            self.succeed = SUCCESS
        # Collision
        elif self.obstacle_distance < THRESHOLD_COLLISION: # obstacle_distance is the minmum distance from LiDAR, if it is below threshold, collision happened
            dynamic_collision = False
            for obstacle_distance in self.obstacle_distances: # iterate dynamic obstacle distances to robot. If a distance is below threshold, it is dynamic obstacle collision (may be improved?) 
                if obstacle_distance < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05): 
                    dynamic_collision = True
            if dynamic_collision:
                self.succeed = COLLISION_OBSTACLE
            else:
                self.succeed = COLLISION_WALL
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.succeed = TIMEOUT
        # Tumble. This activates if the robot falls over. Check because our robot right now is falling often and this may not be working properly.
        elif self.robot_tilt > 0.06 or self.robot_tilt < -0.06:
            self.succeed = TUMBLE
        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)
        return state
    # Intialize the episode
    def initalize_episode(self, response):
        #self.initial_distance_to_goal = self.goal_distance
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.prev_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.diff_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.steps = {'progress': 0, 'since_last_progress': 0, 'total': 0}
        self.exploration = {'current': 0.0, 'previous': 0.0}
        self.processor.lem_map = None
        self.processor.gem_map = None
        response.state = self.get_state(0, 0)
        rw.reward_initialize(None)
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
        #cov_incr = max(0, self.coverage - self.cov_previous)
        #cov_incr = max(0.0, (self.map_known_percent_curr - self.map_known_percent_prev) / 100.0)

        self.steps['count'] = self.local_step
        
        response.reward = float(rw.get_reward_explore(self.succeed, action_linear, action_angular, self.obstacle_distance, self.exploration, self.steps))
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