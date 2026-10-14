
#!/usr/bin/env python3

import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity
from gas_robot_interfaces.msg import GasSensor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DecisionMaker(Node):
    """
    Gas-gradient controller:
    - Takes a short forward measurement step.
    - If gas drops, tests a different heading.
    - Continues searching using gas readings.
    - Stops and marks the source at the critical threshold.
    - Automatically spawns the red gas-source sphere in Gazebo.
    """

    def __init__(self):
        super().__init__('decision_maker')

        # Automatic gas-source sphere spawning
        self.gas_cloud_spawned = False
        self.spawn_cloud_client = self.create_client(
            SpawnEntity, '/spawn_entity'
        )

        # Gas thresholds (ppm)
        self.warning_threshold = 100.0
        self.danger_threshold = 200.0
        self.critical_threshold = 500.0

        # Robot movement settings
        self.forward_speed = 0.08
        self.turn_speed = 0.35
        self.sensor_timeout = 3.0
        self.gas_drop_threshold = 1.0

        # Short movement step before comparing gas concentration
        self.probe_distance = 0.20
        self.position_tolerance = 0.04
        self.gas_samples_needed = 4

        # Sensor values
        self.current_gas = 0.0
        self.temperature = 25.0
        self.humidity = 50.0
        self.sensor_ok = False
        self.last_sensor_time = None
        self.gas_samples = []

        # Robot position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Search state
        self.mode = 'MOVE'
        self.step_start_x = 0.0
        self.step_start_y = 0.0
        self.step_start_gas = None
        self.best_gas = None
        self.best_yaw = 0.0
        self.target_yaw = 0.0
        self.turn_direction = 1.0
        self.last_status_time = 0.0
        self.last_sensor_warning_time = 0.0

        # Source detection state
        self.source_found = False
        self.source_x = None
        self.source_y = None

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.danger_pub = self.create_publisher(
            Point, '/danger_zone', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/visualization_marker', 10
        )

        # Subscribers
        self.create_subscription(
            GasSensor, '/gas_sensor', self.gas_callback, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Control loop
        self.timer = self.create_timer(0.1, self.control_robot)

        self.get_logger().info(
            'Decision Maker started: gradient search enabled; '
            'source threshold >= 500 ppm.'
        )

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.robot_x = pos.x
        self.robot_y = pos.y

        # Convert quaternion to yaw.
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(sin_yaw, cos_yaw)

        self.odom_received = True

    def gas_callback(self, msg):
        self.current_gas = float(msg.gas_ppm)
        self.temperature = float(msg.temperature)
        self.humidity = float(msg.humidity)
        self.sensor_ok = bool(msg.sensor_ok)
        self.last_sensor_time = time.monotonic()

        self.gas_samples.append(self.current_gas)
        if len(self.gas_samples) > self.gas_samples_needed:
            self.gas_samples.pop(0)

    def averaged_gas(self):
        if not self.gas_samples:
            return self.current_gas
        return sum(self.gas_samples) / len(self.gas_samples)

    def gas_level(self):
        if self.current_gas < self.warning_threshold:
            return 'SAFE', 'SEARCHING'
        if self.current_gas < self.danger_threshold:
            return 'WARNING', 'TRACKING'
        if self.current_gas < self.critical_threshold:
            return 'DANGER', 'APPROACHING'
        return 'CRITICAL', 'SOURCE FOUND'

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def detect_source(self):
        if self.source_found:
            return

        self.source_found = True
        self.source_x = self.robot_x
        self.source_y = self.robot_y
        self.mode = 'FOUND'
        self.stop_robot()

        self.get_logger().warning(
            f'SOURCE FOUND: gas={self.current_gas:.2f} ppm, '
            f'position=({self.source_x:.2f}, {self.source_y:.2f}).'
        )

        # New functionality: spawn the sphere at the detected position.
        self.spawn_gas_cloud()

    def spawn_gas_cloud(self):
        if self.gas_cloud_spawned:
            return

        model_path = (
            Path.home()
            / 'ros2_ws'
            / 'src'
            / 'gas_robot_description'
            / 'models'
            / 'red_sphere'
            / 'model.sdf'
        )

        if not model_path.is_file():
            self.get_logger().error(
                f'Gas sphere model not found: {model_path}'
            )
            return

        if not self.spawn_cloud_client.service_is_ready():
            self.get_logger().error(
                'Gazebo /spawn_entity service is not ready. '
                'Cannot spawn gas sphere.'
            )
            return

        request = SpawnEntity.Request()
        request.name = 'red_gas_source'

        with model_path.open('r', encoding='utf-8') as model_file:
            request.xml = model_file.read()

        request.robot_namespace = ''
        request.reference_frame = 'world'

        # Place the sphere at the detected source location.
        request.initial_pose.position.x = float(self.source_x)
        request.initial_pose.position.y = float(self.source_y)
        request.initial_pose.position.z = 1.0
        request.initial_pose.orientation.w = 1.0

        self.gas_cloud_spawned = True

        future = self.spawn_cloud_client.call_async(request)
        future.add_done_callback(self.gas_cloud_spawn_callback)

    def gas_cloud_spawn_callback(self, future):
        try:
            result = future.result()

            if result.success:
                self.get_logger().info(
                    'Red gas-source sphere spawned successfully '
                    f'at ({self.source_x:.2f}, {self.source_y:.2f}).'
                )
            else:
                self.gas_cloud_spawned = False
                self.get_logger().error(
                    f'Could not spawn gas sphere: '
                    f'{result.status_message}'
                )

        except Exception as exc:
            self.gas_cloud_spawned = False
            self.get_logger().error(
                f'Gas sphere spawn request failed: {exc}'
            )

    def publish_danger_point(self):
        point = Point()
        point.x = float(self.source_x)
        point.y = float(self.source_y)
        point.z = 0.0
        self.danger_pub.publish(point)

    def publish_danger_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'danger_zone'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.source_x)
        marker.pose.position.y = float(self.source_y)
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.4

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

    def begin_turn(self, direction):
        self.stop_robot()
        self.mode = 'TURN'
        self.turn_direction = direction
        self.target_yaw = normalize_angle(
            self.robot_yaw + direction * math.pi / 2.0
        )

        self.get_logger().info(
            f'Gas fell; testing another direction from '
            f'({self.robot_x:.2f}, {self.robot_y:.2f}).'
        )

    def control_robot(self):
        now = time.monotonic()

        sensor_stale = (
            self.last_sensor_time is None
            or now - self.last_sensor_time > self.sensor_timeout
        )

        if (
            not self.odom_received
            or sensor_stale
            or not self.sensor_ok
        ):
            self.stop_robot()

            if now - self.last_sensor_warning_time >= 3.0:
                self.last_sensor_warning_time = now
                self.get_logger().warning(
                    'Waiting for odometry and fresh, healthy '
                    'gas sensor data.'
                )
            return

        if self.source_found:
            self.stop_robot()
            self.publish_danger_point()
            self.publish_danger_marker()
            return

        if self.current_gas >= self.critical_threshold:
            self.detect_source()
            self.publish_danger_point()
            self.publish_danger_marker()
            return

        if self.mode == 'MOVE':
            # Compare a smoothed reading after a short movement.
            if self.step_start_gas is None:
                self.step_start_gas = self.averaged_gas()
                self.step_start_x = self.robot_x
                self.step_start_y = self.robot_y

            distance = math.hypot(
                self.robot_x - self.step_start_x,
                self.robot_y - self.step_start_y
            )

            if distance >= self.probe_distance:
                new_gas = self.averaged_gas()
                old_gas = self.step_start_gas
                self.stop_robot()

                self.step_start_gas = new_gas
                self.step_start_x = self.robot_x
                self.step_start_y = self.robot_y

                if new_gas + self.gas_drop_threshold < old_gas:
                    # Turn to test another direction.
                    self.begin_turn(self.turn_direction)
                else:
                    self.mode = 'MOVE'
            else:
                cmd = Twist()
                cmd.linear.x = self.forward_speed
                self.cmd_pub.publish(cmd)

        elif self.mode == 'TURN':
            error = normalize_angle(
                self.target_yaw - self.robot_yaw
            )

            if abs(error) < 0.12:
                self.stop_robot()
                self.mode = 'MOVE'
                self.step_start_gas = self.averaged_gas()
                self.step_start_x = self.robot_x
                self.step_start_y = self.robot_y
                self.turn_direction *= -1.0

                self.get_logger().info(
                    f'Turn complete; checking gas near '
                    f'({self.robot_x:.2f}, {self.robot_y:.2f}).'
                )
            else:
                cmd = Twist()
                cmd.angular.z = (
                    self.turn_speed if error > 0
                    else -self.turn_speed
                )
                self.cmd_pub.publish(cmd)

        if now - self.last_status_time >= 1.0:
            self.last_status_time = now
            level, state = self.gas_level()

            self.get_logger().info(
                f'Position=({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                f'Gas={self.current_gas:.2f} ppm | '
                f'Temperature={self.temperature:.2f} C | '
                f'Humidity={self.humidity:.2f}% | '
                f'Sensor OK={self.sensor_ok} | '
                f'Level={level} | State={state} | Mode={self.mode}'
            )

    def shutdown(self):
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = DecisionMaker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
