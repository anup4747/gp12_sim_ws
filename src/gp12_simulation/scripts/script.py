#!/usr/bin/env python3

import socket
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped


class HyperImuListener(Node):
    """
    Simple node for simulation that listens to Hyper IMU UDP packets
    and republishes the orientation as a ROS topic.

    This is Pink-free and intended only for feeding simulation/visualization.
    """

    def __init__(self) -> None:
        super().__init__('hyper_imu_listener')

        # Parameters
        self.declare_parameter('udp_port', 5555)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('disconnect_timeout_sec', 2.0)

        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.disconnect_timeout_sec = (
            self.get_parameter('disconnect_timeout_sec').get_parameter_value().double_value
        )

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(0.01)

        # Publisher for orientation
        self.orientation_pub = self.create_publisher(QuaternionStamped, 'hyper_imu/orientation', 10)

        # Timer loop
        self.timer = self.create_timer(0.02, self._poll_udp)  # 50 Hz

        self._last_packet_time = None
        self._connected = False
        self._status_timer = self.create_timer(1.0, self._report_connection_status)
        self._packets_received = 0
        self._packets_converted = 0
        self._packets_failed = 0
        self._debug_timer = self.create_timer(2.0, self._debug_report)

        self.get_logger().info(
            f'Hyper IMU listener started on UDP port {self.udp_port}. '
            f'Disconnect timeout: {self.disconnect_timeout_sec:.1f}s'
        )
        self.get_logger().warn(
            'Note: this node only publishes `hyper_imu/orientation`. '
            'It will NOT move the robot unless you run another node that converts IMU data into `/joint_states`.'
        )

    def _poll_udp(self) -> None:
        try:
            data, _ = self.sock.recvfrom(1024)
        except socket.timeout:
            return
        except OSError:
            return

        raw = data.decode('utf-8').strip()
        line = raw.split(',')

        self._last_packet_time = self.get_clock().now()
        self._packets_received += 1

        try:
            values = [float(v) for v in line if v != '']
        except ValueError:
            self._packets_failed += 1
            return

        qx = qy = qz = 0.0
        qw = 1.0
        converted = False

        # Hyper IMU examples we support:
        # - Rotation vector: [rx, ry, rz]  (axis-angle vector, radians)
        # - Quaternion-only: [qx, qy, qz, qw]
        # - Quaternion with prefix fields: [id, time, qx, qy, qz, qw]
        if len(values) == 3:
            rx, ry, rz = values
            angle = math.sqrt(rx * rx + ry * ry + rz * rz)
            if angle <= 1e-12:
                # Zero rotation vector => identity quaternion (valid)
                converted = True
            else:
                ax, ay, az = rx / angle, ry / angle, rz / angle
                s = math.sin(angle / 2.0)
                qx, qy, qz = ax * s, ay * s, az * s
                qw = math.cos(angle / 2.0)
                converted = True
        elif len(values) == 4:
            qx, qy, qz, qw = values
            converted = True
        elif len(values) >= 6:
            qx, qy, qz, qw = values[2:6]
            converted = True
        else:
            self._packets_failed += 1
            return

        if not converted or not all(math.isfinite(v) for v in (qx, qy, qz, qw)):
            self._packets_failed += 1
            return

        # Normalize to be safe (avoid drift if input isn't unit-length).
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n <= 1e-12:
            self._packets_failed += 1
            return
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.quaternion.x = qx
        msg.quaternion.y = qy
        msg.quaternion.z = qz
        msg.quaternion.w = qw

        self.orientation_pub.publish(msg)
        self._packets_converted += 1

    def _report_connection_status(self) -> None:
        now = self.get_clock().now()
        connected = False
        if self._last_packet_time is not None:
            dt = (now - self._last_packet_time).nanoseconds / 1e9
            connected = dt <= self.disconnect_timeout_sec

        if connected != self._connected:
            self._connected = connected
            if connected:
                self.get_logger().info('Hyper IMU sensor connected (UDP data received).')
            else:
                self.get_logger().warn(
                    f'Hyper IMU sensor disconnected (no UDP data for > {self.disconnect_timeout_sec:.1f}s).'
                )

    def _debug_report(self) -> None:
        if self._packets_received == 0:
            self.get_logger().info('Waiting for Hyper IMU UDP packets...')
            return

        self.get_logger().info(
            'Hyper IMU stats: '
            f'received={self._packets_received}, '
            f'converted={self._packets_converted}, '
            f'failed={self._packets_failed}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HyperImuListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()