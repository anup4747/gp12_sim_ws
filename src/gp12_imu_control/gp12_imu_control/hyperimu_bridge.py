import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import math

class HyperIMUBridge(Node):
    def __init__(self):
        super().__init__('hyperimu_bridge')
        
        # Parameters
        self.declare_parameter('udp_port', 5555)
        self.port = self.get_parameter('udp_port').get_parameter_value().integer_value
        
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to all interfaces to receive from phone on the network
        self.sock.bind(('0.0.0.0', self.port))
        # Non-blocking or short timeout so ROS 2 spinner keeps running
        self.sock.settimeout(0.01)
        
        # Timer to read from socket (100Hz max)
        self.timer = self.create_timer(0.01, self.read_udp)
        
        self.get_logger().info(f'HyperIMU Bridge started, listening on UDP port {self.port}')

    def rotation_vector_to_quaternion(self, rx, ry, rz):
        """
        Convert an Android Rotation Vector (rx, ry, rz) to a quaternion.
        The Android rotation vector is defined as an axis-angle representation
        where the direction of the vector is the axis of rotation and the 
        magnitude is sin(theta/2).
        """
        magnitude = math.sqrt(rx**2 + ry**2 + rz**2)
        if magnitude > 0:
            # Android rotation vector magnitude is sin(theta/2)
            # Clip to 1.0 to avoid math domain errors
            sin_half_theta = min(magnitude, 1.0)
            cos_half_theta = math.sqrt(1.0 - sin_half_theta**2)
            
            # Normalize the axis
            ax = rx / magnitude
            ay = ry / magnitude
            az = rz / magnitude
            
            qx = ax * sin_half_theta
            qy = ay * sin_half_theta
            qz = az * sin_half_theta
            qw = cos_half_theta
            return [qx, qy, qz, qw]
        else:
            return [0.0, 0.0, 0.0, 1.0]

    def read_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg_str = data.decode('utf-8').strip()
            
            # User format: ax, ay, az, rx, ry, rz (Rotation Vector)
            parts = msg_str.split(',')
            if len(parts) >= 6:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "imu_link"

                try:
                    # Take last 6 values in case there's an ID or timestamp prefix
                    offset = len(parts) - 6
                    
                    rx = float(parts[offset + 0])
                    ry = float(parts[offset + 1])
                    rz = float(parts[offset + 2])
                    
                    ax = float(parts[offset + 3])
                    ay = float(parts[offset + 4])
                    az = float(parts[offset + 5])
                    
                    # Convert Android rotation vector to quaternion
                    qx, qy, qz, qw = self.rotation_vector_to_quaternion(rx, ry, rz)
                    
                    msg.linear_acceleration.x = ax
                    msg.linear_acceleration.y = ay
                    msg.linear_acceleration.z = az
                    
                    # We don't have gyro data, set an invalid covariance or leave as 0
                    msg.angular_velocity.x = 0.0
                    msg.angular_velocity.y = 0.0
                    msg.angular_velocity.z = 0.0
                    msg.angular_velocity_covariance[0] = -1.0
                    
                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw
                    
                    self.publisher_.publish(msg)
                except ValueError:
                    pass # incomplete read or format mismatch
                
        except socket.timeout:
            pass # No data this cycle
        except Exception as e:
            self.get_logger().error(f'UDP Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HyperIMUBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
