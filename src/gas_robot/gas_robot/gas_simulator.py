import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from gas_robot_interfaces.msg import GasSensor


class GasSimulator(Node):

    def __init__(self):
        super().__init__('gas_simulator')

        # -----------------------------
        # Gas source position
        # -----------------------------
        self.source_x = 2.0
        self.source_y = 2.0

        # Maximum simulated gas strength
        self.gas_strength = 1000.0

        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0

        # -----------------------------
        # Subscriber: robot position
        # -----------------------------
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # -----------------------------
        # Publisher: gas sensor
        # -----------------------------
        self.gas_publisher = self.create_publisher(
            GasSensor,
            '/gas_sensor',
            10
        )

        # Publish gas reading at 5 Hz
        self.timer = self.create_timer(
            0.2,
            self.publish_gas
        )

        self.get_logger().info(
            'Gas simulator started. '
            'Source: (2.0, 2.0)'
        )

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def publish_gas(self):

        # Distance between robot and gas source
        dx = self.robot_x - self.source_x
        dy = self.robot_y - self.source_y

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        # Simulated concentration field
        gas_ppm = self.gas_strength / (
            1.0 + distance * distance
        )

        # Create custom message
        msg = GasSensor()

        msg.gas_ppm = float(gas_ppm)
        msg.temperature = 25.0
        msg.humidity = 50.0
        msg.sensor_ok = True

        self.gas_publisher.publish(msg)

        self.get_logger().info(
            f'Robot: ({self.robot_x:.2f}, '
            f'{self.robot_y:.2f}) | '
            f'Distance: {distance:.2f} m | '
            f'Gas: {gas_ppm:.2f} ppm'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GasSimulator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
