import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserResync(Node):
    def __init__(self):
        super().__init__('laser_resync')

        # Suscriptor al /scan original
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_stable',
            self.callback,
            10
        )

        # Publicador del /scan corregido
        self.pub = self.create_publisher(
            LaserScan,
            '/scan_fixed',
            10
        )

        self.get_logger().info("LaserResync node started. Listening on /scan → Publishing /scan_fixed")

    def callback(self, msg):
        # Copiamos el mensaje
        fixed = LaserScan()
        fixed = msg

        # Reemplazamos el timestamp
        fixed.header.stamp = self.get_clock().now().to_msg()

        # Publicamos
        self.pub.publish(fixed)


def main(args=None):
    rclpy.init(args=args)
    node = LaserResync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
