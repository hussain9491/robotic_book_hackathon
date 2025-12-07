#!/usr/bin/env python3
"""
'Hello Robot' subscriber node
This is a simple ROS 2 subscriber node that demonstrates basic ROS 2 functionality
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloRobotSubscriber(Node):
    """
    A simple subscriber node that subscribes to 'hello_robot' messages
    """

    def __init__(self):
        # Initialize the node with the name 'hello_robot_subscriber'
        super().__init__('hello_robot_subscriber')

        # Create a subscription to String messages on the 'hello_robot' topic
        # with a queue size of 10
        self.subscription = self.create_subscription(
            String,
            'hello_robot',
            self.listener_callback,
            10)

        # Make sure the subscription is not destroyed prematurely
        self.subscription  # prevent unused variable warning

        # Log that the subscriber has started
        self.get_logger().info('Hello Robot Subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function that processes incoming messages
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to run the hello robot subscriber node
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the HelloRobotSubscriber
    hello_robot_subscriber = HelloRobotSubscriber()

    # Run the node until it's shut down
    try:
        rclpy.spin(hello_robot_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node
        hello_robot_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()