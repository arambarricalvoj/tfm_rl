#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LidarPolarViewer(Node):
    def __init__(self):
        super().__init__('lidar_polar_viewer')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_stable',
            self.scan_callback,
            10
        )

        # Crear figura polar
        plt.ion()
        self.fig = plt.figure(figsize=(7,7))
        self.ax = self.fig.add_subplot(111, projection='polar')

        # Configuración de la vista polar
        self.ax.set_theta_zero_location("E")   # 0 rad = eje X positivo (frente)
        self.ax.set_theta_direction(1)        # sentido horario (como ROS)
        self.ax.set_rmax(6.0)                  # alcance máximo típico
        self.ax.grid(True)

        # Marcas de ángulos
        self.ax.set_thetagrids(
            angles=[0, 90, 180, 270],
            labels=["0° Frente", "90° Izquierda", "180° Atrás", "270° Derecha"]
        )

        # Flecha indicando el frente del robot
        self.ax.annotate(
            "", xy=(0, 5.5), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="red", lw=2)
        )
        self.ax.text(0, 5.8, "Frente", color="red", ha="center")

        # Puntos del LiDAR
        self.points_plot, = self.ax.plot([], [], 'b.', markersize=3)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filtrar valores inválidos
        mask = np.isfinite(ranges)
        ranges = ranges[mask]
        angles = angles[mask]

        # Actualizar puntos
        self.points_plot.set_data(angles, ranges)

        self.ax.set_title("Vista polar del LiDAR (0 rad = frente del robot)")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        indices = [0, 125, 250, 375]
        for i in indices:
            print(f"ranges[{i}] = {msg.ranges[i]}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarPolarViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
