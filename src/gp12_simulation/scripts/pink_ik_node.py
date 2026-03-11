#!/usr/bin/env python3

import math
import socket
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pinocchio as pin
import pink
from pink import solve_ik
from pink.tasks import FrameTask


def _rotation_vector_to_quaternion(rx: float, ry: float, rz: float) -> pin.Quaternion:
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle <= 1e-12:
        return pin.Quaternion(1.0, 0.0, 0.0, 0.0)
    ax, ay, az = rx / angle, ry / angle, rz / angle
    s = math.sin(angle / 2.0)
    return pin.Quaternion(math.cos(angle / 2.0), ax * s, ay * s, az * s)


class PinkIkNode(Node):
    def __init__(self) -> None:
        super().__init__('pink_ik_node')

        # Parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('udp_port', 5555)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('publish_topic', '/joint_states')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('disconnect_timeout_sec', 2.0)

        self.urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.disconnect_timeout_sec = (
            self.get_parameter('disconnect_timeout_sec').get_parameter_value().double_value
        )

        if not self.urdf_path:
            raise RuntimeError('Parameter `urdf_path` is empty. Provide a valid URDF path.')

        # Load Pinocchio model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        q0 = pin.neutral(self.model)
        self.configuration = pink.Configuration(self.model, self.data, q0)

        # Task: track end-effector orientation from IMU; keep position fixed at initial pose
        self.task = FrameTask(self.ee_frame, position_cost=0.0, orientation_cost=1.0)
        self.tasks = [self.task]

        tf0 = self.configuration.get_transform(self.base_frame, self.ee_frame)
        self._target_pos = tf0.translation.copy()
        self._target_quat_xyzw = pin.Quaternion(tf0.rotation).coeffs().copy()

        # ROS publisher
        self.joint_pub = self.create_publisher(JointState, self.publish_topic, 10)

        # UDP thread state
        self._running = True
        self._lock = threading.Lock()
        self._last_packet_time = None
        self._connected = False

        self._udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._udp_thread.start()

        # Timers
        dt = 1.0 / max(1.0, self.rate_hz)
        self._timer = self.create_timer(dt, self._update_ik_and_publish)
        self._status_timer = self.create_timer(1.0, self._report_connection_status)

        self.get_logger().info(
            f'Pink IK node started. UDP port={self.udp_port}, ee_frame={self.ee_frame}, rate={self.rate_hz:.1f}Hz'
        )

    def destroy_node(self):
        self._running = False
        try:
            if self._udp_thread.is_alive():
                self._udp_thread.join(timeout=1.0)
        finally:
            return super().destroy_node()

    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.udp_port))
        sock.settimeout(0.5)

        while self._running:
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            raw = data.decode('utf-8').strip()
            parts = [p for p in raw.split(',') if p != '']
            try:
                values = [float(v) for v in parts]
            except ValueError:
                continue

            # Hyper IMU rotation vector: [rx, ry, rz]
            if len(values) == 3:
                rx, ry, rz = values
                quat = _rotation_vector_to_quaternion(rx, ry, rz)
                coeffs = quat.coeffs()  # x,y,z,w
                with self._lock:
                    self._target_quat_xyzw = np.array(coeffs, dtype=float)
                    self._last_packet_time = self.get_clock().now()
                continue

            # If user switches to quaternion output:
            if len(values) == 4:
                qx, qy, qz, qw = values
                with self._lock:
                    self._target_quat_xyzw = np.array([qx, qy, qz, qw], dtype=float)
                    self._last_packet_time = self.get_clock().now()
                continue

            if len(values) >= 6:
                qx, qy, qz, qw = values[2:6]
                with self._lock:
                    self._target_quat_xyzw = np.array([qx, qy, qz, qw], dtype=float)
                    self._last_packet_time = self.get_clock().now()

    def _update_ik_and_publish(self) -> None:
        # Read current target quaternion
        with self._lock:
            qx, qy, qz, qw = self._target_quat_xyzw.tolist()

        # Normalize quaternion
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n <= 1e-12:
            return
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

        target_pose = pin.SE3(pin.Quaternion(qw, qx, qy, qz).toRotationMatrix(), self._target_pos)
        self.task.set_target(target_pose)

        dt = 1.0 / max(1.0, self.rate_hz)
        try:
            velocity = solve_ik(self.configuration, self.tasks, dt, solver='osqp')
        except Exception:
            # Fallback if osqp isn't installed/configured
            velocity = solve_ik(self.configuration, self.tasks, dt)

        self.configuration.integrate(velocity, dt)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.model.names[i] for i in range(1, self.model.njoints)]
        msg.position = self.configuration.q.tolist()
        self.joint_pub.publish(msg)

    def _report_connection_status(self) -> None:
        now = self.get_clock().now()
        connected = False
        with self._lock:
            last = self._last_packet_time
        if last is not None:
            dt = (now - last).nanoseconds / 1e9
            connected = dt <= self.disconnect_timeout_sec

        if connected != self._connected:
            self._connected = connected
            if connected:
                self.get_logger().info('Hyper IMU connected (driving Pink IK).')
            else:
                self.get_logger().warn(
                    f'Hyper IMU disconnected (no UDP data for > {self.disconnect_timeout_sec:.1f}s).'
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PinkIkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

