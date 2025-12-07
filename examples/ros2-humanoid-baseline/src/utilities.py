"""
Common utility functions for ROS 2 node initialization
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def initialize_ros_node(node_name: str):
    """
    Initialize a ROS 2 node with proper setup

    Args:
        node_name (str): Name of the ROS 2 node

    Returns:
        Node: Initialized ROS 2 node instance
    """
    if not rclpy.ok():
        rclpy.init()

    node = Node(node_name)
    return node


def create_qos_profile(reliability: str = 'reliable',
                    durability: str = 'volatile',
                    history: str = 'keep_last',
                    depth: int = 10):
    """
    Create a QoS profile for ROS 2 communications

    Args:
        reliability (str): 'reliable' or 'best_effort'
        durability (str): 'volatile' or 'transient_local'
        history (str): 'keep_last' or 'keep_all'
        depth (int): History depth for keep_last

    Returns:
        QoSProfile: Configured QoS profile
    """
    reliability_policy = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT
    durability_policy = DurabilityPolicy.VOLATILE if durability == 'volatile' else DurabilityPolicy.TRANSIENT_LOCAL

    qos_profile = QoSProfile(
        depth=depth,
        reliability=reliability_policy,
        durability=durability_policy
    )

    return qos_profile


def shutdown_ros_node(node: Node):
    """
    Properly shutdown a ROS 2 node

    Args:
        node (Node): The ROS 2 node to shutdown
    """
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def wait_for_message(node: Node, msg_type, topic_name: str, timeout_sec: float = 5.0):
    """
    Wait for a single message on a topic

    Args:
        node (Node): The ROS 2 node to use for subscription
        msg_type: The message type (e.g., String, Int32, etc.)
        topic_name (str): The topic to listen on
        timeout_sec (float): Maximum time to wait in seconds

    Returns:
        The received message or None if timeout
    """
    message_received = None
    received = False

    def callback(msg):
        nonlocal message_received, received
        message_received = msg
        received = True

    subscription = node.create_subscription(
        msg_type,
        topic_name,
        callback,
        10
    )

    start_time = node.get_clock().now()
    while not received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
            break

    node.destroy_subscription(subscription)
    return message_received