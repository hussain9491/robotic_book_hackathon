#!/usr/bin/env python3
"""
Workspace validation script for the hello robot example
This script tests that the hello robot publisher and subscriber work correctly
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time


class HelloRobotValidator(Node):
    """
    A validation node that tests the hello robot publisher/subscriber functionality
    """

    def __init__(self):
        super().__init__('hello_robot_validator')

        # Variables to track validation results
        self.messages_received = []
        self.message_count = 0
        self.max_messages = 5  # Number of messages to wait for

        # Create subscription to verify publisher is working
        self.subscription = self.create_subscription(
            String,
            'hello_robot',
            self.validator_callback,
            10)

        self.subscription  # prevent unused variable warning

        # Log that validation is starting
        self.get_logger().info('Hello Robot Validator node initialized')
        self.get_logger().info(f'Waiting for {self.max_messages} messages from hello_robot publisher...')

    def validator_callback(self, msg):
        """
        Callback function that receives messages from hello robot publisher
        """
        self.messages_received.append(msg.data)
        self.message_count += 1
        self.get_logger().info(f'Validator received: "{msg.data}"')

        # Stop after receiving the required number of messages
        if self.message_count >= self.max_messages:
            self.get_logger().info(f'Validation successful! Received {self.message_count} messages.')
            rclpy.shutdown()


def validate_hello_robot_setup():
    """
    Validates that the hello robot publisher and subscriber are working correctly
    """
    print("Starting hello robot setup validation...")

    # Initialize ROS 2
    rclpy.init()

    # Create validator node
    validator = HelloRobotValidator()

    # Run the validator node
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print("Validation interrupted by user")
        return False

    # Check results
    success = validator.message_count >= validator.max_messages
    print(f"Validation {'PASSED' if success else 'FAILED'}")
    print(f"Messages received: {validator.message_count}/{validator.max_messages}")

    # Clean up
    validator.destroy_node()

    return success


def main():
    """
    Main function to run the validation
    """
    print("Running workspace validation for hello robot example...")
    print("="*60)

    success = validate_hello_robot_setup()

    print("="*60)
    if success:
        print("✓ Workspace validation PASSED - Hello Robot example is working correctly!")
        return 0
    else:
        print("✗ Workspace validation FAILED - Hello Robot example is not working properly")
        return 1


if __name__ == '__main__':
    exit(main())