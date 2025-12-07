#!/usr/bin/env python3
"""
Joint control service server for humanoid robot
Provides a service to control specific joints of the humanoid robot
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from std_msgs.msg import Float64MultiArray
from example_interfaces.srv import SetBool, Trigger


class JointControlServer(Node):
    """
    Service server that handles joint control commands for a humanoid robot
    """

    def __init__(self):
        # Initialize the node with the name 'joint_control_server'
        super().__init__('joint_control_server')

        # Create a callback group for services to allow concurrent handling
        self.callback_group = ReentrantCallbackGroup()

        # Create service for setting a single joint position
        self.single_joint_srv = self.create_service(
            SetBool,  # Using SetBool as a simple example; in practice, you'd create a custom service
            'set_joint_position',
            self.set_single_joint_callback,
            callback_group=self.callback_group
        )

        # For demonstration purposes, create a custom service-like functionality
        # using a more appropriate service type for joint control
        self.joint_command_srv = self.create_service(
            SetBool,  # This would normally be a custom service like 'SetJointCommand'
            'command_joint',
            self.joint_command_callback,
            callback_group=self.callback_group
        )

        # Create a service for emergency stop
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

        # Log that the service server has started
        self.get_logger().info('Joint Control Service Server initialized')
        self.get_logger().info(f'Available joints: {list(self.current_positions.keys())}')

    def set_single_joint_callback(self, request, response):
        """
        Callback for setting a single joint position
        In a real implementation, this would take joint name and position as parameters
        """
        # For this example, we'll just return success
        # In a real implementation, you'd parse the request.data to extract joint name and position
        response.success = True
        response.message = "Single joint command received (simulated)"
        self.get_logger().info(f'Single joint command processed: {request.data}')
        return response

    def joint_command_callback(self, request, response):
        """
        Callback for joint command service
        This would typically receive joint name and target position
        """
        # Simulate processing the joint command
        response.success = True
        response.message = "Joint command processed successfully"
        self.get_logger().info(f'Joint command processed: {request.data}')
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

    # Create an instance of the JointControlServer
    joint_control_server = JointControlServer()

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