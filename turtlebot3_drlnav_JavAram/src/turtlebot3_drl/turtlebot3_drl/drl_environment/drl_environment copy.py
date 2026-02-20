import math
import numpy
import sys
import copy
from math import inf as Infinity
import time

from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import DrlStep, RingGoal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy

from . import reward as rw
from ..common import utilities as util
from ..common.settings import (
    ENABLE_BACKWARD,
    EPISODE_TIMEOUT_SECONDS,
    ENABLE_MOTOR_NOISE,
    UNKNOWN,
    SUCCESS,
    COLLISION_WALL,
    COLLISION_OBSTACLE,
    TIMEOUT,
    TUMBLE,
    TOPIC_SCAN,
    TOPIC_VELO,
    TOPIC_ODOM,
    ARENA_LENGTH,
    ARENA_WIDTH,
    MAX_NUMBER_OBSTACLES,
    OBSTACLE_RADIUS,
    LIDAR_DISTANCE_CAP,
    SPEED_LINEAR_MAX,
    SPEED_ANGULAR_MAX,
    THRESHOLD_COLLISION,
    ENABLE_DYNAMIC_GOALS,
    IG_SUCCESS_THRESHOLD
)

NUM_SCAN_SAMPLES = 40
LINEAR = 0
ANGULAR = 1


class DRLEnvironment(Node):
    def __init__(self):
        super().__init__('drl_environment')

        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}")

        self.episode_timeout = EPISODE_TIMEOUT_SECONDS

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.robot_tilt = 0.0
        self.robot_x_prev = 0.0
        self.robot_y_prev = 0.0
        self.total_distance = 0.0

        # Episode state
        self.done = False
        self.succeed = UNKNOWN
        self.episode_deadline = Infinity
        self.reset_deadline = False
        self.clock_msgs_skipped = 0
        self.local_step = 0
        self.time_sec = 0

        # SLAM sync
        self.waiting_first_map = True

        # Obstacles
        self.obstacle_distances = [Infinity] * MAX_NUMBER_OBSTACLES

        # LiDAR
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP

        # Map / entropy / coverage
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.entropy_prev = 0.0
        self.entropy_current = 0.0
        self.coverage_prev = 0.0
        self.coverage_current = 0.0

        # NEW: frontier list
        self.frontiers = []

        # NEW: gradient angle
        self.angle_to_gradient = 0.0

        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(depth=1)
        qos_clock.reliability = ReliabilityPolicy.BEST_EFFORT

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, TOPIC_VELO, qos)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, TOPIC_ODOM, self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub = self.create_subscription(Odometry, 'obstacle/odom', self.obstacle_odom_callback, qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)

        # Gazebo reset services
        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')

        # Step service
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)

    # -------------------------------------------------------------------------
    # MAP CALLBACK — detect frontiers + compute entropy/coverage
    # -------------------------------------------------------------------------
    def map_callback(self, msg):
        if self.waiting_first_map:
            print("[ENV] Primer mapa recibido → episodio puede comenzar")
            self.waiting_first_map = False

        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        total_cells = self.map_width * self.map_height
        if total_cells == 0:
            return

        data = msg.data

        unknown_count = 0
        known_count = 0

        # NEW: frontier detection
        self.frontiers = []
        w = self.map_width
        h = self.map_height

        for idx, v in enumerate(data):
            if v == -1:
                unknown_count += 1
                continue
            else:
                known_count += 1

            # Check if this known cell has an unknown neighbor
            row = idx // w
            col = idx % w

            neighbors = [
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1)
            ]

            is_frontier = False
            for nr, nc in neighbors:
                if 0 <= nr < h and 0 <= nc < w:
                    n_idx = nr * w + nc
                    if data[n_idx] == -1:
                        is_frontier = True
                        break

            if is_frontier:
                wx = self.map_origin_x + (col + 0.5) * self.map_resolution
                wy = self.map_origin_y + (row + 0.5) * self.map_resolution
                self.frontiers.append((wx, wy))

        # Entropy and coverage
        self.entropy_prev = self.entropy_current
        self.entropy_current = unknown_count / total_cells

        self.coverage_prev = self.coverage_current
        self.coverage_current = known_count / total_cells

        if self.local_step % 20 == 0:
            print(f"[MAP] cov: {self.coverage_current:.3f} ent: {self.entropy_current:.3f} frontiers: {len(self.frontiers)}")

    # -------------------------------------------------------------------------
    # ODOM / SCAN / CLOCK / OBSTACLES
    # -------------------------------------------------------------------------
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

        if self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2)
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y

    def scan_callback(self, msg):
        self.obstacle_distance = 1.0
        for i in range(NUM_SCAN_SAMPLES):
            self.scan_ranges[i] = numpy.clip(float(msg.ranges[i]) / LIDAR_DISTANCE_CAP, 0, 1)
            if self.scan_ranges[i] < self.obstacle_distance:
                self.obstacle_distance = self.scan_ranges[i]
        self.obstacle_distance *= LIDAR_DISTANCE_CAP

    def clock_callback(self, msg):
        self.time_sec = msg.clock.sec
        if not self.reset_deadline:
            return
        self.clock_msgs_skipped += 1
        if self.clock_msgs_skipped <= 10:
            return
        episode_time = self.episode_timeout
        if ENABLE_DYNAMIC_GOALS:
            episode_time = numpy.clip(episode_time * 1.0, 10, 50)
        self.episode_deadline = self.time_sec + episode_time
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

    def obstacle_odom_callback(self, msg):
        if 'obstacle' in msg.child_frame_id:
            robot_pos = msg.pose.pose.position
            obstacle_id = int(msg.child_frame_id[-1]) - 1
            dx = self.robot_x - robot_pos.x
            dy = self.robot_y - robot_pos.y
            self.obstacle_distances[obstacle_id] = math.sqrt(dx*dx + dy*dy)

    # -------------------------------------------------------------------------
    # STATE
    # -------------------------------------------------------------------------
    def get_state(self, prev_lin, prev_ang):
        state = copy.deepcopy(self.scan_ranges)
        state.append(float(prev_lin))
        state.append(float(prev_ang))
        self.local_step += 1

        # Grace period
        if self.local_step <= 50:
            return state

        # Success by exploration
        if self.coverage_current >= IG_SUCCESS_THRESHOLD and self.succeed is UNKNOWN:
            self.succeed = SUCCESS

        # Collisions
        if self.succeed is UNKNOWN:
            if self.obstacle_distance < THRESHOLD_COLLISION:
                dyn = any(d < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05)
                          for d in self.obstacle_distances)
                self.succeed = COLLISION_OBSTACLE if dyn else COLLISION_WALL

            elif self.time_sec >= self.episode_deadline:
                self.succeed = TIMEOUT

            elif abs(self.robot_tilt) > 0.06:
                self.succeed = TUMBLE

        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)

        return state

    # -------------------------------------------------------------------------
    # RESET
    # -------------------------------------------------------------------------
    def stop_reset_robot(self, success):
        self.cmd_vel_pub.publish(Twist())
        self.episode_deadline = Infinity
        self.done = True

        req = RingGoal.Request()
        req.robot_pose_x = self.robot_x
        req.robot_pose_y = self.robot_y
        req.radius = 1.0

        client = self.task_succeed_client if success else self.task_fail_client
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        client.call_async(req)

        self.waiting_first_map = True

        time.sleep(0.5)

    # -------------------------------------------------------------------------
    # EPISODE INIT
    # -------------------------------------------------------------------------
    def initalize_episode(self, response):
        self.entropy_prev = 0.0
        self.entropy_current = 0.0
        self.coverage_prev = 0.0
        self.coverage_current = 0.0

        self.angle_to_gradient = 0.0
        self.frontiers = []

        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0

        rw.reward_initalize(0.0)
        return response

    # -------------------------------------------------------------------------
    # STEP
    # -------------------------------------------------------------------------
    def step_comm_callback(self, request, response):

        if self.waiting_first_map:
            response.state = self.get_state(0, 0)
            response.reward = 0.0
            response.done = False
            return response

        if len(request.action) == 0:
            return self.initalize_episode(response)

        if ENABLE_MOTOR_NOISE:
            request.action[LINEAR] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)
            request.action[ANGULAR] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)

        # Denormalize
        if ENABLE_BACKWARD:
            action_linear = request.action[LINEAR] * SPEED_LINEAR_MAX
        else:
            action_linear = (request.action[LINEAR] + 1) / 2 * SPEED_LINEAR_MAX
        action_angular = request.action[ANGULAR] * SPEED_ANGULAR_MAX

        # Publish cmd_vel
        twist = Twist()
        twist.linear.x = action_linear
        twist.angular.z = action_angular
        self.cmd_vel_pub.publish(twist)

        # ---------------------------------------------------------------------
        # NEW: Compute gradient direction from frontiers
        # ---------------------------------------------------------------------
        Gx = 0.0
        Gy = 0.0

        """if self.coverage_current > 0.005 and len(self.frontiers) > 30:
            # usar gradiente
        else:
            self.angle_to_gradient = 0.0
        """

        if len(self.frontiers) > 0 and self.coverage_current > 0.001:
            for fx, fy in self.frontiers:
                dx = fx - self.robot_x
                dy = fy - self.robot_y
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < 0.3:
                    continue

                weight = 1.0 / (dist * dist)
                Gx += weight * dx
                Gy += weight * dy

            if abs(Gx) > 1e-6 or abs(Gy) > 1e-6:
                self.angle_to_gradient = math.atan2(Gy, Gx) - self.robot_heading
                self.angle_to_gradient = (self.angle_to_gradient + math.pi) % (2 * math.pi) - math.pi
            else:
                self.angle_to_gradient = 0.0
        else:
            self.angle_to_gradient = 0.0

        # Next state
        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR])

        # Reward
        base_reward = rw.get_reward_exploration(
            self.succeed,
            action_linear,
            action_angular,
            self.obstacle_distance,
            self.entropy_prev,
            self.entropy_current,
            self.angle_to_gradient
        )

        response.reward = float(base_reward)
        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0

        if self.done:
            response.distance_traveled = self.total_distance
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False
            self.reset_deadline = True

        if self.local_step % 20 == 0:
            print(
                f"R: {response.reward:<8.4f} "
                f"MinD: {self.obstacle_distance:<6.2f} "
                f"Alin: {request.action[LINEAR]:<6.2f} "
                f"Aturn: {request.action[ANGULAR]:<6.2f} "
                f"cov: {self.coverage_current:.3f} "
                f"ent: {self.entropy_current:.3f} "
                f"ang_grad: {math.degrees(self.angle_to_gradient):.1f}° "
                f"frontiers: {len(self.frontiers)}"
            )

        return response


def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    drl_environment = DRLEnvironment()
    rclpy.spin(drl_environment)
    drl_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()