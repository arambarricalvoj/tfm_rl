#!/usr/bin/env python3

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
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

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
    MAX_NUMBER_OBSTACLES,
    LIDAR_DISTANCE_CAP,
    SPEED_LINEAR_MAX,
    SPEED_ANGULAR_MAX,
    THRESHOLD_COLLISION,
    OBSTACLE_RADIUS
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
        self.robot_x_prev = 0.0
        self.robot_y_prev = 0.0
        self.robot_heading = 0.0
        self.robot_tilt = 0.0
        self.total_distance = 0.0

        # Episode state
        self.done = False
        self.succeed = UNKNOWN
        self.episode_deadline = Infinity
        self.reset_deadline = False
        self.clock_msgs_skipped = 0
        self.episode_started = False

        # Obstacles
        self.obstacle_distances = [Infinity] * MAX_NUMBER_OBSTACLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP

        # LiDAR
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES

        # Exploration
        self.map_coverage = 0.0
        self.prev_coverage = 0.0
        self.last_coverage = 0.0
        self.last_coverage_time = 0.0
        self.coverage_threshold = 0.85
        self.coverage_stagnation_seconds = 5.0

        self.local_step = 0
        self.time_sec = 0

        # SLAM sync
        self.slam_ready = False

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # ROS setup
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(depth=1)
        qos_clock.reliability = ReliabilityPolicy.BEST_EFFORT

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, TOPIC_VELO, qos, callback_group=self.cb_group
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, TOPIC_ODOM, self.odom_callback, qos, callback_group=self.cb_group
        )

        self.scan_sub = self.create_subscription(
            LaserScan, TOPIC_SCAN, self.scan_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.cb_group
        )

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_clock, callback_group=self.cb_group
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos, callback_group=self.cb_group
        )

        self.obstacle_odom_sub = self.create_subscription(
            Odometry, 'obstacle/odom', self.obstacle_odom_callback,
            qos, callback_group=self.cb_group
        )

        # Services
        self.task_succeed_client = self.create_client(
            RingGoal, 'task_succeed', callback_group=self.cb_group
        )
        self.task_fail_client = self.create_client(
            RingGoal, 'task_fail', callback_group=self.cb_group
        )

        self.slam_ready_srv = self.create_service(
            Trigger, 'slam_ready', self.slam_ready_callback,
            callback_group=self.cb_group
        )

        self.step_comm_server = self.create_service(
            DrlStep, 'step_comm', self.step_comm_callback,
            callback_group=self.cb_group
        )

    # -----------------------------
    # Callbacks
    # -----------------------------

    def slam_ready_callback(self, request, response):
        self.slam_ready = True
        self.get_logger().info("Environment: SLAM ready signal received")
        response.success = True
        response.message = "SLAM ready acknowledged"
        return response

    def obstacle_odom_callback(self, msg):
        if 'obstacle' in msg.child_frame_id:
            pos = msg.pose.pose.position
            idx = int(msg.child_frame_id[-1]) - 1
            dx = self.robot_x - pos.x
            dy = self.robot_y - pos.y
            self.obstacle_distances[idx] = math.sqrt(dx*dx + dy*dy)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

        if self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2
            )
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y

    def scan_callback(self, msg):
        self.obstacle_distance = 1
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

        self.episode_deadline = self.time_sec + self.episode_timeout
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

    def map_callback(self, msg: OccupancyGrid):
        data = msg.data
        total = len(data)
        if total == 0:
            return
        known = sum(1 for c in data if c != -1)
        self.map_coverage = known / total

        if self.map_coverage > self.last_coverage:
            self.last_coverage = self.map_coverage
            self.last_coverage_time = self.time_sec
        
        self.get_logger().info(f"Map received. Coverage: {self.map_coverage*100:.1f}%")

    # -----------------------------
    # Reset
    # -----------------------------

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
            self.get_logger().info('Waiting for Gazebo Goals service...')
        client.call_async(req)

        self.episode_started = False

        # Esperar a SLAM listo SOLO entre episodios
        self.slam_ready = False
        self.get_logger().info("Environment: waiting for SLAM ready signal...")

        start = time.time()
        while not self.slam_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 30:
                self.get_logger().warn("Timeout waiting for SLAM ready")
                break

        time.sleep(0.2)

    # -----------------------------
    # State + reward
    # -----------------------------

    def get_state(self, prev_lin, prev_ang):
        state = copy.deepcopy(self.scan_ranges)
        state.append(float(prev_lin))
        state.append(float(prev_ang))
        self.local_step += 1

        # Grace period
        if self.local_step <= 30:
            return state

        if not self.episode_started:
            return state

        # MIN STEPS BEFORE TERMINATION
        if self.local_step < 80:
            return state

        # Collision
        if self.obstacle_distance < THRESHOLD_COLLISION:
            dynamic_collision = False
            for d in self.obstacle_distances:
                if d < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05):
                    dynamic_collision = True
            self.succeed = COLLISION_OBSTACLE if dynamic_collision else COLLISION_WALL

        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.succeed = TIMEOUT

        # Tumble
        elif self.robot_tilt > 0.06 or self.robot_tilt < -0.06:
            self.succeed = TUMBLE

        # Coverage success
        elif self.map_coverage >= self.coverage_threshold:
            self.succeed = SUCCESS

        # Stagnation
        elif (
            self.map_coverage > 0.5 and
            (self.time_sec - self.last_coverage_time) > self.coverage_stagnation_seconds
        ):
            self.succeed = SUCCESS

        if self.succeed is not UNKNOWN and self.episode_started:

            """print(
                f"[EPISODE END] "
                f"MinD: {self.obstacle_distance:<5.2f}  "
                f"Map: {self.map_coverage*100:5.1f}%  "
                f"Steps: {self.local_step}"
            )"""

            self.stop_reset_robot(self.succeed == SUCCESS)

        return state

    def initalize_episode(self, response):

        # NO esperar a slam_ready aquí
        # slam_ready solo se usa entre episodios

        self.prev_coverage = self.map_coverage
        self.last_coverage = self.map_coverage
        self.last_coverage_time = self.time_sec

        self.episode_started = True

        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0

        return response

    # -----------------------------
    # Step
    # -----------------------------

    def step_comm_callback(self, request, response):
        if len(request.action) == 0:
            self.local_step = 0
            return self.initalize_episode(response)

        if ENABLE_MOTOR_NOISE:
            request.action[LINEAR] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)
            request.action[ANGULAR] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)

        if ENABLE_BACKWARD:
            action_linear = request.action[LINEAR] * SPEED_LINEAR_MAX
        else:
            action_linear = (request.action[LINEAR] + 1) / 2 * SPEED_LINEAR_MAX

        action_angular = request.action[ANGULAR] * SPEED_ANGULAR_MAX

        twist = Twist()
        twist.linear.x = action_linear
        twist.angular.z = action_angular
        self.cmd_vel_pub.publish(twist)

        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR])

        response.reward = float(rw.get_reward(
            self.succeed,
            action_linear,
            action_angular,
            0.0,
            0.0,
            self.obstacle_distance,
            self.map_coverage,
            self.prev_coverage,
            "explore"
        ))

        self.prev_coverage = self.map_coverage

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

        if self.local_step % 200 == 0:
            print(
                f"R: {response.reward:<6.2f}  "
                f"MinD: {self.obstacle_distance:<5.2f}  "
                f"Alin: {request.action[LINEAR]:<5.2f}  "
                f"Aturn: {request.action[ANGULAR]:<5.2f}  "
                f"Map: {self.map_coverage*100:5.1f}%"
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
