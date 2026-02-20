# reset_core.py
import time
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity, SpawnEntity


def move_robot(node, linear_x=0.0, angular_z=0.0, duration=0.1):
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    node.cmd_vel_pub.publish(msg)
    time.sleep(duration)
    node.cmd_vel_pub.publish(Twist())


def reset_simulation(node):
    # 1. Parar robot
    move_robot(node, 0.0, 0.0, 0.5)

    # 2. Resetear física de Gazebo
    while not node.reset_world_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Esperando /reset_world...')
    node.reset_world_client.call_async(Empty.Request())

    # 3. Borrar goal
    delete_entity(node, node.entity_name)

    # 4. Respawnear goal
    spawn_entity(node, node.entity_name, node.entity, node.goal_x, node.goal_y)

    time.sleep(0.2)


def delete_entity(node, name):
    req = DeleteEntity.Request()
    req.name = name
    node.delete_entity_client.call_async(req)


def spawn_entity(node, name, xml, x, y):
    req = SpawnEntity.Request()
    req.name = name
    req.xml = xml
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    req.initial_pose = pose
    node.spawn_entity_client.call_async(req)
