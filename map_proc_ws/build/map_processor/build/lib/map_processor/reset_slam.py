import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import signal
import os

class SlamManager(Node):
    def __init__(self):
        super().__init__('slam_manager')

        self.slam_cmd = [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=True',
            'slam_params_file:=create3_ws/src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml'
        ]

        self.slam_process = None
        self.start_slam()

        self.srv = self.create_service(Trigger, 'reset_slam', self.reset_slam_callback)

    def start_slam(self):
        self.get_logger().info("Iniciando SLAM Toolbox...")
        self.slam_process = subprocess.Popen(self.slam_cmd, preexec_fn=os.setsid)

    def stop_slam(self):
        if self.slam_process is not None:
            self.get_logger().info("Matando proceso SLAM Toolbox...")
            try:
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.slam_process.wait()
            self.slam_process = None

    def reset_slam_callback(self, request, response):
        self.get_logger().warn("Reiniciando SLAM Toolbox por petición del servicio...")
        self.stop_slam()
        self.start_slam()
        response.success = True
        response.message = "SLAM Toolbox reiniciado correctamente"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C detectado desde main()")
    finally:
        node.stop_slam()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
