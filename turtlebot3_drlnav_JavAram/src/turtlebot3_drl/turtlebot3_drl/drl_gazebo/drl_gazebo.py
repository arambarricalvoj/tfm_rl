#!/usr/bin/env python3
#
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

import os
import random
import math
import numpy
import time
import math

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

#from gazebo_msgs.srv import SetEntityState
#from gazebo_msgs.msg import EntityState

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from turtlebot3_msgs.srv import RingGoal
import xml.etree.ElementTree as ET
from ..drl_environment.drl_environment import ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS
from ..common.settings import ENABLE_TRUE_RANDOM_GOALS

from rclpy.task import Future
import tf2_ros

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid


NO_GOAL_SPAWN_MARGIN = 0.3 # meters away from any wall

class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.robot_name = "create3"
        self.robot_urdf_path = "/home/raul/create3_ws/src/create3_sim/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro"
 
        self.entity_dir_path = (os.path.dirname(os.path.realpath(__file__))).replace(
            'turtlebot3_drl/lib/python3.10/site-packages/turtlebot3_drl/drl_gazebo',
            'turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_drl_world/goal_box')
        self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goal'

        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}, dynamic goals enabled: {ENABLE_DYNAMIC_GOALS}")

        self.prev_x, self.prev_y = -1, -1
        self.goal_x, self.goal_y = 0.5, 0.0

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        # Initialise publishers
        # self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', QoSProfile(depth=10))
        self.cmd_vel_pub   = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialise client
        self.delete_entity_client       = self.create_client(DeleteEntity, 'delete_entity')
        # self.spawn_entity_client        = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client    = self.create_client(Empty, 'reset_simulation')
        self.reset_world_client         = self.create_client(Empty, '/reset_world')
        self.gazebo_pause               = self.create_client(Empty, '/pause_physics')

        # Initialise servers
        self.task_succeed_server    = self.create_service(RingGoal, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server       = self.create_service(RingGoal, 'task_fail', self.task_fail_callback)

        self.obstacle_coordinates   = self.get_obstacle_coordinates()
        self.init_callback()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""      
    def init_callback(self):
       self.delete_entity()
       self.reset_simulation()

       self.publish_callback()
       print("Init, goal pose:", self.goal_x, self.goal_y)
       time.sleep(1)
       
    def publish_callback(self):
        # Publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        # self.goal_pose_pub.publish(goal_pose)
        # self.spawn_entity()

    def task_succeed_callback(self, request, response):
        self.delete_entity()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"success: generate (random) a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def task_fail_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"fail: reset the environment, (random) goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def goal_is_valid(self, goal_x, goal_y):
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0]:
                if goal_y < obstacle[0][1] and goal_y > obstacle[2][1]:
                    return False
        return True

    def generate_random_goal(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0
        while (((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 4) or (not self.goal_is_valid(self.goal_x, self.goal_y))):
            self.goal_x = random.randrange(-25, 25) / 10.0
            self.goal_y = random.randrange(-25, 25) / 10.0
            tries += 1
            if tries > 200:
                print("ERROR: cannot find valid new goal, resestting!")
                self.delete_entity()
                self.reset_simulation()
                self.generate_goal_pose()
                break
        self.publish_callback()

    def generate_dynamic_goal_pose(self, robot_pose_x, robot_pose_y, radius):
        tries = 0
        while(True):
            ring_position = random.uniform(0, 1)
            origin = radius + numpy.random.normal(0, 0.1) # in meters
            goal_offset_x = math.cos(2 * math.pi * ring_position) * origin
            goal_offset_y = math.sin(2 * math.pi * ring_position) * origin
            goal_x = robot_pose_x + goal_offset_x
            goal_y = robot_pose_y + goal_offset_y
            if self.goal_is_valid(goal_x, goal_y):
                self.goal_x = goal_x
                self.goal_y = goal_y
                break
            if tries > 100:
                print("Error! couldn't find valid goal position, resetting..")
                self.delete_entity()
                self.reset_simulation()
                self.generate_goal_pose()
                return
            tries += 1
        self.publish_callback()


    def generate_goal_pose(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0

        while ((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 2):
            if self.stage == 11:
                # --- Define static goal positions here ---
                goal_pose_list = [[0.0, 0.0], [0.0, 6.5], [5.0, 5.5], [-2.5, -6.0], [3.0, -4.0], [6.0, -1.0]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            elif self.stage == 8 or self.stage == 9 or self.stage == 12:
                # --- Define static goal positions here ---
                goal_pose_list = [[2.0, 2.0], [2.0, 1.5], [2.0, -0.5], [2.0, -1.0], [2.0, -2.0], [1.3, 1.0],
                                    [1.0, 0.3], [1.0, -2.0], [0.3, -1.0],  [0.0, 2.0], [0.0, -1.0], [-1.0, 1.0],
                                        [-1.0, -1.2], [-2.0, 1.0], [-2.2, 0.0], [-2.0, -2.2], [-2.4, 2.4]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            elif self.stage not in [4, 5, 7]:
                self.goal_x = random.randrange(-15, 16) / 10.0
                self.goal_y = random.randrange(-15, 16) / 10.0
            else:
                # --- Define static goal positions here ---
                goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                                  [-1.9, 1.9], [-1.9,  0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0],
                                  [1.5, -1.0], [-0.5, 1.0], [-1.0, -2.0], [1.8, -0.2], [1.0, -1.9]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            tries += 1
            if tries > 100:
                print("ERROR: distance between goals is small!")
                break
        # Add here fixed goal if needed
        #self.goal_x = float(2.0)
        #self.goal_y = float(0.0)
        self.publish_callback()

    def nodoa_kudeatu(self, nodoa, transition_id, next_step=None):
        self.get_logger().info(f'/{nodoa}/change_state → {transition_id}')

        client = self.create_client(ChangeState, f'/{nodoa}/change_state')

        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'{nodoa}/change_state no disponible, reintentando...')
            self.create_timer(0.5, lambda: self.nodoa_kudeatu(nodoa, transition_id, next_step))
            return

        request = ChangeState.Request()
        request.transition.id = transition_id

        future = client.call_async(request)

        def done_cb(fut):
            if fut.result() is not None:
                self.get_logger().info(f'Transición OK: {nodoa} → {transition_id}')
            else:
                self.get_logger().error(f'Error en transición: {nodoa} → {transition_id}')

            if next_step is not None:
                next_step()

        future.add_done_callback(done_cb)


    # -------------------------
    # NUEVO: Pausar Gazebo
    # -------------------------
    def pause_gazebo(self, next_step=None):
        self.pause_physics = self.create_client(Empty, '/pause_physics')
        self.pause_physics.call_async(Empty.Request())

        if not self.pause_physics.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('pause_physics no disponible, reintentando...')
            self.create_timer(0.5, lambda: self.pause_gazebo(next_step))
            return

        future = self.pause_physics.call_async(Empty.Request())

        def done_cb(fut):
            self.get_logger().info("Gazebo PAUSADO.")
            if next_step:
                next_step()

        future.add_done_callback(done_cb)


    def unpause_gazebo(self):
        self.unpause_physics = self.create_client(Empty, '/unpause_physics')
        self.unpause_physics.call_async(Empty.Request())

        if not self.unpause_physics.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('unpause_physics no disponible, reintentando...')
            self.create_timer(0.5, self.unpause_gazebo)
            return

        self.unpause_physics.call_async(Empty.Request())
        self.get_logger().info("Gazebo REANUDADO.")


    # -------------------------
    # RESET SIMULATION
    # -------------------------
    def reset_simulation(self):
        # Parar robot
        self.move_robot(linear_x=0.0, angular_z=0.0, duration=0.5)

        # 1) DEACTIVATE
        self.nodoa_kudeatu(
            'slam_toolbox',
            Transition.TRANSITION_DEACTIVATE,
            next_step=lambda: self.nodoa_kudeatu(
                'slam_toolbox',
                Transition.TRANSITION_CLEANUP,
                next_step=self._reset_simulation_async
            )
        )


    def _reset_simulation_async(self):
        req = Empty.Request()
        future = self.reset_world_client.call_async(req)

        def done_cb(fut):
            self.get_logger().info("RESET SIMULATION completado.")
            self._configure_and_activate()

        future.add_done_callback(done_cb)


    def _configure_and_activate(self):
        # 4) CONFIGURE
        self.nodoa_kudeatu(
            'slam_toolbox',
            Transition.TRANSITION_CONFIGURE,
            next_step=self._episode_ready
        )


    def _episode_ready(self):
        self.get_logger().info("Episodio listo: SLAM sincronizado y Gazebo reseteado.")

    
    def move_robot(self, linear_x=0.0, angular_z=0.0, duration=0.1):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        time.sleep(duration)
        self.cmd_vel_pub.publish(Twist())  # detener
        self.get_logger().info("Movimiento completado.")


    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.delete_entity_client.call_async(req)

    """def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_entity_client.call_async(req)"""

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
            point_3 = [point_1[0] - size_x, point_1[1] - size_y ]
            point_4 = [point_1[0] - size_x, point_1[1] ]
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
