#!/usr/bin/env python3
"""
Joint control service server for humanoid robot (real implementation)
Provides a service to control specific joints of the humanoid robot
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from example_interfaces.srv import SetBool  # Using SetBool as we don't have custom .srv in basic ROS 2
import time


class JointControlServerReal(Node):
    """
    Service server that handles joint control commands for a humanoid robot
    Using standard ROS 2 services since custom .srv files require package building
    """

    def __init__(self):
        # Initialize the node with the name 'joint_control_server_real'
        super().__init__('joint_control_server_real')

        # Create a callback group for services to allow concurrent handling
        self.callback_group = ReentrantCallbackGroup()

        # Create service for commanding joint positions using a SetBool service
        # In a real implementation, you would create a custom .srv file
        self.joint_command_srv = self.create_service(
            SetBool,
            'command_joint_position',
            self.joint_command_callback,
            callback_group=self.callback_group
        )

        # Create service for emergency stop
        self.emergency_stop_srv = self.create_service(
            Trigger,
            'emergency_stop',
            self.emergency_stop_callback,
            callback_group=self.callback_group
        )

        # Store current joint positions (simulated)
        self.current_positions = {
            'head_pan_joint': 0.0,
            'left_shoulder_pitch': 0.0,
            'left_shoulder_roll': 0.0,
            'left_elbow_joint': 0.0,
            'left_wrist_joint': 0.0,
            'right_shoulder_pitch': 0.0,
            'right_shoulder_roll': 0.0,
            'right_elbow_joint': 0.0,
            'right_wrist_joint': 0.0,
            'left_hip_pitch': 0.0,
            'left_hip_roll': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_pitch': 0.0,
            'right_hip_roll': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0
        }

        # Store joint limits for safety
        self.joint_limits = {
            'head_pan_joint': (-1.57, 1.57),
            'left_shoulder_pitch': (-2.0, 2.0),
            'left_shoulder_roll': (-1.57, 1.57),
            'left_elbow_joint': (-2.0, 0.0),
            'left_wrist_joint': (-1.0, 1.0),
            'right_shoulder_pitch': (-2.0, 2.0),
            'right_shoulder_roll': (-1.57, 1.57),
            'right_elbow_joint': (-2.0, 0.0),
            'right_wrist_joint': (-1.0, 1.0),
            'left_hip_pitch': (-1.0, 1.0),
            'left_hip_roll': (-0.5, 0.5),
            'left_knee_joint': (0.0, 2.0),
            'left_ankle_joint': (-0.5, 0.5),
            'right_hip_pitch': (-1.0, 1.0),
            'right_hip_roll': (-0.5, 0.5),
            'right_knee_joint': (0.0, 2.0),
            'right_ankle_joint': (-0.5, 0.5)
        }

        # Log that the service server has started
        self.get_logger().info('Joint Control Service Server (Real) initialized')
        self.get_logger().info(f'Available joints: {len(self.current_positions)} joints')

    def joint_command_callback(self, request, response):
        """
        Callback for joint command service
        In a real implementation, this would parse joint name and position from request
        For this example, we'll simulate processing based on request data
        """
        # In a real implementation, you would parse a custom message that includes:
        # - joint name
        # - target position
        # - max effort

        # For simulation, we'll just return success
        response.success = True
        response.message = "Joint command processed (simulated)"

        self.get_logger().info(f'Received joint command request: {request.data}')
        return response

    def emergency_stop_callback(self, request, response):
        """
        Callback for emergency stop service
        """
        # Simulate emergency stop - reset all joints to safe positions
        for joint_name in self.current_positions:
            self.current_positions[joint_name] = 0.0

        response.success = True
        response.message = "Emergency stop activated - all joints set to safe positions"
        self.get_logger().warn('EMERGENCY STOP ACTIVATED - All joints set to safe positions')
        return response


def main(args=None):
    """
    Main function to run the joint control service server
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the JointControlServerReal
    joint_control_server = JointControlServerReal()

    # Use a multi-threaded executor to handle multiple service calls concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(joint_control_server)

    try:
        # Spin the node to handle service requests
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node
        joint_control_server.destroy_node()
        rclpy.shutdown()
        executor.shutdown()


if __name__ == '__main__':
    main()