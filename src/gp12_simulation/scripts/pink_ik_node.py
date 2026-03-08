#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import pinocchio as pin
import pink
from pink import solve_ik
from pink.tasks import FrameTask
import socket
import threading

class PinkIKNode(Node):
    def __init__(self):
        super().__init__('pink_ik_node')
        
        # Parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('udp_port', 5555)
        self.declare_parameter('ee_frame', 'tool0')
        
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        
        if not urdf_path:
            self.get_logger().error("URDF path is empty!")
            return

        # Load Pinocchio model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.configuration = pink.Configuration(self.model, self.data, self.model.referenceConfigurations["default"] if "default" in self.model.referenceConfigurations else pin.neutral(self.model))
        
        # IK Task
        self.task = FrameTask(self.ee_frame, position_cost=1.0, orientation_cost=1.0)
        self.tasks = [self.task]
        
        # ROS 2 Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # State
        self.target_pos = self.configuration.get_transform('base_link', self.ee_frame).translation.copy()
        self.target_quat = pin.Quaternion(self.configuration.get_transform('base_link', self.ee_frame).rotation).coeffs().copy()
        
        # UDP Thread
        self.running = True
        self.initial_pos = self.target_pos.copy()
        self.udp_thread = threading.Thread(target=self.udp_listener)
        self.udp_thread.start()
        
        # Timer for IK and Publishing
        self.timer = self.create_timer(0.01, self.update_ik) # 100Hz
        
        self.get_logger().info(f"Pink IK Node started. Listening on UDP port {self.udp_port}")

    def udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.udp_port))
        sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                line = data.decode('utf-8').strip()
                parts = line.split(',')
                
                # Based on the image showing 9 columns:
                # [acc_x, acc_y, acc_z, rot_x, rot_y, rot_z, acc_x, acc_y, acc_z]
                if len(parts) >= 6:
                    # Accelerometer (1-3)
                    # We use this as a relative position offset from the initial IK position
                    # Phone Z is usually pointing out of the screen, X to the right, Y up
                    ax = float(parts[0]) 
                    ay = float(parts[1])
                    az = float(parts[2]) - 9.81 # Remove gravity
                    
                    # Apply a small smoothing/integration or just direct mapping for demo
                    # Let's map phone position (rough approximation)
                    self.target_pos[0] = self.initial_pos[0] + ax * 0.05
                    self.target_pos[1] = self.initial_pos[1] + ay * 0.05
                    self.target_pos[2] = self.initial_pos[2] + az * 0.05

                    # Rotation vector (4-6) - Assuming Euler angles (radians)
                    rx = float(parts[3])
                    ry = float(parts[4])
                    rz = float(parts[5])
                    
                    # Create rotation matrix from Euler angles
                    # Adjust order (e.g. 'xyz') depending on phone orientation
                    R = pin.utils.rpyToMatrix(rx, ry, rz)
                    self.target_quat = pin.Quaternion(R).coeffs()

            except socket.timeout:
                continue
            except Exception as e:
                pass # Silent error for high frequency

    def update_ik(self):
        # Update task target
        target_pose = pin.SE3(pin.Quaternion(self.target_quat).toRotationMatrix(), self.target_pos)
        self.task.set_target(target_pose)
        
        # Solve IK
        dt = 0.01
        velocity = solve_ik(self.configuration, self.tasks, dt, solver='osqp')
        self.configuration.integrate(velocity, dt)
        
        # Publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # We MUST publish all joints found in the URDF to avoid TF errors
        # model.names[0] is 'universe', we skip it.
        msg.name = [self.model.names[i] for i in range(1, self.model.njoints)]
        
        # config.q includes values for these joints.
        msg.position = self.configuration.q.tolist()
        self.joint_pub.publish(msg)

    def stop(self):
        self.running = False
        self.udp_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = PinkIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
