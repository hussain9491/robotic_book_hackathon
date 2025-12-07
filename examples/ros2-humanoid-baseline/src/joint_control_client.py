#!/usr/bin/env python3
"""
Joint control service client for humanoid robot
Client that sends joint control commands to the service server
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.srv import Trigger, SetBool
import time
import sys


class JointControlClient(Node):
    """
    Service client that sends joint control commands to the humanoid robot
    """

    def __init__(self):
        # Initialize the node with the name 'joint_control_client'
        super().__init__('joint_control_client')

        # Create clients for the services
        self.joint_command_client = self.create_client(
            SetBool,  # Using SetBool as a placeholder; real implementation would use custom service
            'command_joint_position'
        )

        self.emergency_stop_client = self.create_client(
            Trigger,
            'emergency_stop'
        )

        # Wait for services to be available
        self.get_logger().info('Waiting for services to become available...')
        self.joint_command_client.wait_for_service(timeout_sec=5.0)
        self.emergency_stop_client.wait_for_service(timeout_sec=5.0)

        # Log that the client is ready
        self.get_logger().info('Joint Control Client initialized and connected to services')

    def send_joint_command(self, command_data=True):
        """
        Send a joint command to the server

        Args:
            command_data: Command data to send (in real implementation, this would include joint name and position)

        Returns:
            Response from the service
        """
        # Create the request
        request = SetBool.Request()
        request.data = command_data

        # Call the service asynchronously
        future = self.joint_command_client.call_async(request)
        self.get_logger().info(f'Sent joint command request: {command_data}')

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None:
            self.get_logger().info(f'Joint command response: success={response.success}, message="{response.message}"')
            return response
        else:
            self.get_logger().error('Failed to get response from joint command service')
            return None

    def send_emergency_stop(self):
        """
        Send an emergency stop command to the server

        Returns:
            Response from the service
        """
        # Create the request
        request = Trigger.Request()

        # Call the service asynchronously
        future = self.emergency_stop_client.call_async(request)
        self.get_logger().warn('Sending EMERGENCY STOP command...')

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None:
            self.get_logger().info(f'Emergency stop response: success={response.success}, message="{response.message}"')
            return response
        else:
            self.get_logger().error('Failed to get response from emergency stop service')
            return None


def main(args=None):
    """
    Main function to run the joint control client
    Demonstrates how to use the service client
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the JointControlClient
    joint_control_client = JointControlClient()

    # Create an executor
    executor = SingleThreadedExecutor()
    executor.add_node(joint_control_client)

    try:
        # Example 1: Send a joint command
        joint_control_client.get_logger().info("Example 1: Sending joint command")
        response = joint_control_client.send_joint_command(True)
        if response:
            print(f"Command result: {response.success} - {response.message}")

        # Wait a bit
        time.sleep(1)

        # Example 2: Send another joint command
        joint_control_client.get_logger().info("Example 2: Sending another joint command")
        response = joint_control_client.send_joint_command(False)
        if response:
            print(f"Command result: {response.success} - {response.message}")

        # Wait a bit
        time.sleep(1)

        # Example 3: Send emergency stop (just for demonstration - don't actually do this!)
        # Uncomment the next lines if you want to test emergency stop
        # joint_control_client.get_logger().info("Example 3: Sending emergency stop (for demonstration)")
        # response = joint_control_client.send_emergency_stop()
        # if response:
        #     print(f"Emergency stop result: {response.success} - {response.message}")

        print("Service client examples completed successfully!")

    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node
        joint_control_client.destroy_node()
        rclpy.shutdown()
        executor.shutdown()


if __name__ == '__main__':
    main()