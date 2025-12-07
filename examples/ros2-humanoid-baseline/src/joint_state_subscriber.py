#!/usr/bin/env python3
"""
Joint state subscriber node for humanoid robot
Subscribes to joint states and logs the information
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class JointStateSubscriber(Node):
    """
    Subscriber node that receives joint states for a humanoid robot
    """

    def __init__(self):
        # Initialize the node with the name 'joint_state_subscriber'
        super().__init__('joint_state_subscriber')

        # Create a subscription to JointState messages on the 'joint_states' topic
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Make sure the subscription is not destroyed prematurely
        self.subscription  # prevent unused variable warning

        # Track time for rate limiting log output
        self.last_log_time = time.time()
        self.log_interval = 1.0  # Log every 1 second

        # Log that the subscriber has started
        self.get_logger().info('Joint State Subscriber node initialized')

    def joint_state_callback(self, msg):
        """
        Callback function that processes incoming joint state messages
        """
        current_time = time.time()

        # Rate limit logging to avoid overwhelming output
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time

            # Log joint information
            self.get_logger().info(f'Received joint state message with {len(msg.name)} joints')

            # Log the first few joint positions as an example
            if len(msg.name) > 0:
                for i in range(min(3, len(msg.name))):  # Log first 3 joints
                    joint_name = msg.name[i]
                    position = msg.position[i] if i < len(msg.position) else 0.0
                    self.get_logger().info(f'  {joint_name}: {position:.3f} rad')

            # Log if we have more joints than we're displaying
            if len(msg.name) > 3:
                self.get_logger().info(f'  ... and {len(msg.name) - 3} more joints')


def main(args=None):
    """
    Main function to run the joint state subscriber node
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the JointStateSubscriber
    joint_state_subscriber = JointStateSubscriber()

    # Run the node until it's shut down
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()