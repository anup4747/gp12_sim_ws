import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Imu
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import numpy as np

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]

class ImuToMoveitNode(Node):
    def __init__(self):
        super().__init__('imu_to_moveit')
        
        # Subscribe to IMU data from the phone
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Action client for MoveIt's MoveGroup action
        self.move_action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Wait for the action server
        self.get_logger().info('Waiting for /move_action server...')
        self.move_action_client.wait_for_server()
        self.get_logger().info('/move_action server is ready.')

        # Target link and group name
        self.planning_group = "arm"
        self.end_effector_link = "tool0"
        
        # State tracking
        self.base_orientation = None
        self.current_target_pose = self.get_initial_pose()
        
        # Rate limit how often we send goals to MoveIt
        self.timer = self.create_timer(1.0, self.plan_and_execute_timer)
        self.latest_imu_msg = None
        
        self.action_in_progress = False

    def get_initial_pose(self):
        # A safe resting pose for the GP12 above its base
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.8
        # End effector pointing roughly forwards/downwards
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        return pose

    def imu_callback(self, msg):
        self.latest_imu_msg = msg

    def plan_and_execute_timer(self):
        if self.latest_imu_msg is None:
            return
            
        if self.action_in_progress:
            return # Wait until current motion finishes
            
        imu_quat = [
            self.latest_imu_msg.orientation.w,
            self.latest_imu_msg.orientation.x,
            self.latest_imu_msg.orientation.y,
            self.latest_imu_msg.orientation.z
        ]
        
        # In a real app we'd calibrate this. 
        # For simplicity, we just apply the phone's absolute orientation to the robot.
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position = self.current_target_pose.pose.position
        
        # Map Android IMU orientation directly for test
        target_pose.pose.orientation.w = imu_quat[0]
        target_pose.pose.orientation.x = imu_quat[1]
        target_pose.pose.orientation.y = imu_quat[2]
        target_pose.pose.orientation.z = imu_quat[3]
        
        self.send_moveit_goal(target_pose)

    def send_moveit_goal(self, target_pose):
        self.action_in_progress = True
        goal_msg = MoveGroup.Goal()
        
        # Tell MoveIt to use the current robot state as the start state
        goal_msg.request.start_state.is_diff = True
        
        # Set planning group
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 1.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Construct orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = self.end_effector_link
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        
        # Construct position constraint holding the EE at the start position
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = self.end_effector_link
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.1] # 10cm tolerance box to allow some wiggle room for IK
        
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0
        
        constraint = Constraints()
        constraint.name = "imu_target"
        constraint.orientation_constraints.append(oc)
        constraint.position_constraints.append(pc)
        
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.min_corner.z = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.0
        
        goal_msg.request.goal_constraints.append(constraint)
        
        self.get_logger().info(f"Sending goal targeting orientation {target_pose.pose.orientation}")
        
        send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by MoveIt!')
            self.action_in_progress = False
            return

        self.get_logger().info('Goal accepted, executing trajectory...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == 1:
            self.get_logger().info('Success! Trajectory executed.')
        else:
            self.get_logger().warn(f'MoveIt failed with error code: {error_code}')
            
        self.action_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = ImuToMoveitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
