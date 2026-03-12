# map_processor/map_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

# importa la clase que implementaste
from .map_processing_utils import MapProcessor

class SlamListener(Node):
    def __init__(self):
        super().__init__('slam_listener')

        # ---- inicializar MapProcessor ----
        # puedes pasar parámetros opcionales si la clase los acepta (umbral, etc.)
        self.processor = MapProcessor()

        try:
            self.processor.start_plot()
        except Exception as e:
            self.get_logger().warn(f'No se pudo iniciar plot: {e}')

        # suscripciones
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_cb, 10)

        # estado local
        self.current_map_msg = None
        self.current_pose = None

    def map_cb(self, msg: OccupancyGrid):
        # actualizar el objeto MapProcessor con el OccupancyGrid recibido
        try:
            self.current_map_msg = msg
            self.processor.update_from_occupancy_grid(msg)
            self.processor.update_plot()
        except Exception as e:
            self.get_logger().error(f'Error actualizando mapa: {e}')
            return

        # acceder a métricas ya calculadas y loguearlas o publicarlas
        free = self.processor.free_count
        occ = self.processor.occupied_count
        known_pct = 100 - self.processor.unknown_percent
        coverage = self.processor.free_ratio_known  # o el nombre que uses
        bbox = self.processor.occupied_bbox
        w = self.processor.width if self.processor.width is not None else 'N/A'
        h = self.processor.height if self.processor.height is not None else 'N/A'

        self.get_logger().info(
            f'Map updated: free={free} occ={occ} known%={known_pct:.1f} '
            f'free_ratio_known={coverage:.2f} bbox={bbox} '
            f'H={h} W={w}'
        )

        # ejemplo: usar otras funciones del procesador
        # dist_map = self.processor.compute_distance_transform()  # si lo implementas

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        # guardamos el mensaje si lo necesitas
        self.current_pose = msg.pose.pose
        # actualizamos el MapProcessor con la pose completa (extrae yaw internamente)
        try:
            self.processor.set_robot_pose_from_pose_msg(msg)
            self.processor.update_plot()
        except Exception as e:
            self.get_logger().error(f'Error al setear robot pose: {e}')
            return

        p = self.current_pose.position
        self.get_logger().info(f'Pose recibida: x={p.x:.2f}, y={p.y:.2f}')
        # opcional: leer la celda y su valor
        cell = self.processor.get_robot_cell()
        val = self.processor.get_robot_cell_value()
        self.get_logger().info(f'Robot cell: {cell}, value: {val}')


def main(args=None):
    rclpy.init(args=args)
    node = SlamListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
