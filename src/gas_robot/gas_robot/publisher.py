import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from gas_robot_interfaces.msg import GasSensor


class GasSensorPublisher(Node):

    def __init__(self):
        super().__init__('gas_sensor_publisher')

        self.publisher_ = self.create_publisher(
            GasSensor,
            'gas_sensor_data',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.timer = self.create_timer(
            1.0,
            self.publish_sensor_data
        )

        self.source_x = 5.0
        self.source_y = 0.0

        self.get_logger().info(
            'Gas Sensor Publisher started'
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def publish_sensor_data(self):

        msg = GasSensor()

        # Calculate distance from robot to gas source
        distance = math.sqrt(
            (self.robot_x - self.source_x) ** 2 +
            (self.robot_y - self.source_y) ** 2
        )

        # Simulate gas concentration based on distance
        self.gas_ppm = 20.0 + 480.0 * math.exp(
            -(distance ** 2) / 4.0
        )

        msg.gas_ppm = self.gas_ppm
        msg.temperature = 27.5
        msg.humidity = 60.0
        msg.sensor_ok = True

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Gas: {msg.gas_ppm:.1f} ppm | '
            f'Temp: {msg.temperature:.1f} C | '
            f'Humidity: {msg.humidity:.1f}% | '
            f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
            f'Sensor OK: {msg.sensor_ok}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GasSensorPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()