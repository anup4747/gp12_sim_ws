import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class ManualJog(Node):
    def __init__(self):
        super().__init__('gp12_manual_jog')

        # ROS parameters
        self.declare_parameter('topic', '/servo_server/delta_twist_cmds')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('vx_max', 0.04)      # m/s
        self.declare_parameter('vy_max', 0.04)      # m/s
        self.declare_parameter('x_frequency', 0.3)  # Hz
        self.declare_parameter('y_frequency', 0.2)  # Hz
        self.declare_parameter('loop_time_sec', 30.0)
        self.declare_parameter('phase_shift', math.pi / 2.0)

        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.x_frequency = float(self.get_parameter('x_frequency').value)
        self.y_frequency = float(self.get_parameter('y_frequency').value)
        self.loop_time_sec = float(self.get_parameter('loop_time_sec').value)
        self.phase_shift = float(self.get_parameter('phase_shift').value)

        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            'Starting real-time servo XY jogging: '
            f'vx_max={self.vx_max} m/s, vy_max={self.vy_max} m/s, '
            f'x_freq={self.x_frequency} Hz, y_freq={self.y_frequency} Hz, '
            f'frame_id={self.frame_id}, topic={topic}'
        )

    def tick(self):
        elapsed = time.monotonic() - self.start_time

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Real-time Cartesian velocity pattern in the XY plane.
        # This is not MoveIt planning; it is continuous servo control.
        t = elapsed
        vx = self.vx_max * math.sin(2.0 * math.pi * self.x_frequency * t)
        vy = self.vy_max * math.sin(2.0 * math.pi * self.y_frequency * t + self.phase_shift)

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)

        # stop after a fixed test window if requested
        if self.loop_time_sec > 0.0 and elapsed >= self.loop_time_sec:
            self.get_logger().info(
                f'XY servo test finished after {self.loop_time_sec} seconds; sending zero velocity.'
            )
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            self.pub.publish(msg)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ManualJog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
