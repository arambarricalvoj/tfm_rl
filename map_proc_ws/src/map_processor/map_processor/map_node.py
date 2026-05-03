# map_processor/map_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

from .map_processing_utils import MapProcessor


class SlamListener(Node):
    def __init__(self):
        super().__init__('slam_listener')

        # ---- inicializar MapProcessor ----
        self.processor = MapProcessor()

        # ---- Activar LEM y GEM ----
        self.processor.enable_lem = False
        self.processor.enable_global_reduced_map = False
        self.processor.enable_global_reduced_occ_map = True
        self.processor.lem_scale = 3

        # ---- Iniciar visualización ----
        try:
            self.processor.start_plot()
        except Exception as e:
            self.get_logger().warn(f'No se pudo iniciar plot: {e}')

        # ---- Suscripciones ----
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_cb, 10)

        self.current_map_msg = None
        self.current_pose = None


    # ---------------------------------------------------------
    #   CALLBACK DE MAPA
    # ---------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        try:
            self.current_map_msg = msg
            self.processor.update_from_occupancy_grid(msg)
            self.processor.update_plot()
        except Exception as e:
            self.get_logger().error(f'Error actualizando mapa: {e}')
            return

        # métricas
        free = self.processor.free_count
        occ = self.processor.occupied_count
        known_pct = 100 - self.processor.unknown_percent
        coverage = self.processor.free_ratio_known
        bbox = self.processor.explored_bbox
        w = self.processor.width
        h = self.processor.height

        self.get_logger().info(
            f'Map updated: free={free} occ={occ} known%={known_pct:.1f} '
            f'coverage={coverage:.2f} bbox={bbox} H={h} W={w}'
        )

        # EJEMPLO: acceder a LEM, GEM y CNN
        # lem = self.processor.lem_map
        # gem = self.processor.gem_map
        # cnn = self.processor.cnn_map


    # ---------------------------------------------------------
    #   CALLBACK DE POSE
    # ---------------------------------------------------------
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

        try:
            self.processor.set_robot_pose_from_pose_msg(msg)
            self.processor.update_plot()
        except Exception as e:
            self.get_logger().error(f'Error al setear robot pose: {e}')
            return

        p = self.current_pose.position
        self.get_logger().info(f'Pose recibida: x={p.x:.2f}, y={p.y:.2f}')

        cell = self.processor.get_robot_cell()
        val = self.processor.get_robot_cell_value()
        self.get_logger().info(f'Robot cell: {cell}, value: {val}')


def main(args=None):
    rclpy.init(args=args)
    node = SlamListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
