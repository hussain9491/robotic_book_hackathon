#!/usr/bin/env python3
"""
Joint state publisher node for humanoid robot
Publishes joint states for a humanoid robot with arms, legs, and torso
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import random


class JointStatePublisher(Node):
    """
    Publisher node that publishes joint states for a humanoid robot
    """

    def __init__(self):
        # Initialize the node with the name 'joint_state_publisher'
        super().__init__('joint_state_publisher')

        # Create a publisher for JointState messages on the 'joint_states' topic
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Create a timer to publish messages at 50 Hz (0.02 seconds)
        timer_period = 0.02  # seconds (50 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize joint names for a simple humanoid model
        self.joint_names = [
            'head_pan_joint',           # Head rotation
            'left_shoulder_pitch',      # Left shoulder up/down
            'left_shoulder_roll',       # Left shoulder in/out
            'left_elbow_joint',         # Left elbow
            'left_wrist_joint',         # Left wrist
            'right_shoulder_pitch',     # Right shoulder up/down
            'right_shoulder_roll',      # Right shoulder in/out
            'right_elbow_joint',        # Right elbow
            'right_wrist_joint',        # Right wrist
            'left_hip_pitch',           # Left hip up/down
            'left_hip_roll',            # Left hip in/out
            'left_knee_joint',          # Left knee
            'left_ankle_joint',         # Left ankle
            'right_hip_pitch',          # Right hip up/down
            'right_hip_roll',           # Right hip in/out
            'right_knee_joint',         # Right knee
            'right_ankle_joint'         # Right ankle
        ]

        # Initialize joint positions (starting positions)
        self.joint_positions = [0.0] * len(self.joint_names)

        # Initialize joint velocities (starting at rest)
        self.joint_velocities = [0.0] * len(self.joint_names)

        # Initialize joint efforts (no initial forces)
        self.joint_efforts = [0.0] * len(self.joint_names)

        # Time tracking for generating periodic motion
        self.time_counter = 0.0

        # Log that the publisher has started
        self.get_logger().info(f'Joint State Publisher node initialized with {len(self.joint_names)} joints')

    def timer_callback(self):
        """
        Callback function that publishes joint state messages
        """
        # Create a JointState message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names
        msg.name = self.joint_names

        # Update joint positions with some periodic motion to make it interesting
        self.time_counter += 0.02  # Increment by timer period

        # Create some simple oscillating motion for demonstration
        for i in range(len(self.joint_positions)):
            # Add some periodic motion based on joint index
            amplitude = 0.2 + (i % 3) * 0.1  # Different amplitude for each joint
            frequency = 0.5 + (i % 5) * 0.2  # Different frequency for each joint
            self.joint_positions[i] = amplitude * math.sin(frequency * self.time_counter)

            # Add some small random variation
            self.joint_positions[i] += random.uniform(-0.05, 0.05)

        # Set the joint state values
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        # Publish the message
        self.publisher_.publish(msg)

        # Log a message periodically to show activity
        if int(self.time_counter) % 5 == 0 and self.time_counter % 0.02 == 0:
            self.get_logger().info(f'Published joint states for {len(self.joint_names)} joints')


def main(args=None):
    """
    Main function to run the joint state publisher node
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the JointStatePublisher
    joint_state_publisher = JointStatePublisher()

    # Run the node until it's shut down
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()