#!/usr/bin/env python3
"""
'Hello Robot' publisher node
This is a simple ROS 2 publisher node that demonstrates basic ROS 2 functionality
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloRobotPublisher(Node):
    """
    A simple publisher node that publishes 'Hello Robot' messages
    """

    def __init__(self):
        # Initialize the node with the name 'hello_robot_publisher'
        super().__init__('hello_robot_publisher')

        # Create a publisher for String messages on the 'hello_robot' topic
        # with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'hello_robot', 10)

        # Create a timer to publish messages every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track the number of messages published
        self.i = 0

        # Log that the publisher has started
        self.get_logger().info('Hello Robot Publisher node initialized')

    def timer_callback(self):
        """
        Callback function that publishes a message every timer tick
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello Robot! Message #{self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to run the hello robot publisher node
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the HelloRobotPublisher
    hello_robot_publisher = HelloRobotPublisher()

    # Run the node until it's shut down
    try:
        rclpy.spin(hello_robot_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node
        hello_robot_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()