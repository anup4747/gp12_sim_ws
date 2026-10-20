import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Manual jog node: publishes a short linear motion command on a ROS topic.
class ManualJog(Node):
    def __init__(self):
        super().__init__("gp12_manual_jog")

        # Parameters that can be configured externally via launch or ROS params.
        self.declare_parameter(
            "topic", "/servo_server/delta_twist_cmds"
        )
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("speed", 0.01)
        self.declare_parameter("duration", 1.0)

        # Read the parameter values into instance variables.
        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.speed = float(self.get_parameter("speed").value)
        self.duration = float(self.get_parameter("duration").value)

        # Create the publisher for TwistStamped messages.
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.start_time = time.monotonic()

        # Run the motion loop at 50 Hz.
        self.timer = self.create_timer(0.02, self.tick)

        # Log the intended motion command for debugging.
        self.get_logger().info(
            f"Jogging X at {self.speed} m/s for "
            f"{self.duration} seconds on {topic}"
        )

    def tick(self):
        # Compute how long the node has been running since start.
        elapsed = time.monotonic() - self.start_time

        # Build the message payload for each update cycle.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Move forward while the duration window is active; otherwise send zero velocity.
        if elapsed < self.duration:
            msg.twist.linear.x = self.speed
        else:
            # Publish zero velocity before exiting.
            msg.twist.linear.x = 0.0

        # Publish the current command to the servo controller.
        self.pub.publish(msg)

        # Stop the node after a short post-move settling period.
        if elapsed >= self.duration + 0.3:
            self.get_logger().info("Jog complete; zero velocity sent.")
            rclpy.shutdown()


def main(args=None):
    # Initialize ROS and create the node instance.
    rclpy.init(args=args)
    node = ManualJog()

    try:
        # Keep the node alive and process callbacks until shutdown.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown if ROS is still active.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
