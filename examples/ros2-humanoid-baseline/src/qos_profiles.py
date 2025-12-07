"""
QoS (Quality of Service) profile configurations for real-time communication
Used in the Physical AI & Humanoid Robotics educational examples
"""

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


class QoSProfiles:
    """
    A collection of pre-configured QoS profiles for different types of ROS 2 communication
    """

    # For sensor data that needs to be delivered reliably with low latency
    SENSOR_DATA = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    # For joint state information that needs real-time delivery
    JOINT_STATES = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    # For control commands that must be delivered reliably
    CONTROL_COMMANDS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    # For emergency stop commands that must be delivered immediately
    EMERGENCY_COMMANDS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        # lifespan=Duration(seconds=1)  # Message expires after 1 second
    )

    # For periodic status updates that don't need to be stored
    STATUS_UPDATES = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    # For configuration parameters that should persist
    PARAMETERS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
    )

    # For high-frequency sensor data where some loss is acceptable
    HIGH_FREQUENCY_SENSOR = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    # For logging and debugging information
    LOGGING = QoSProfile(
        history=QoSHistoryPolicy.KEEP_ALL,
        depth=100,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )


def get_qos_profile(profile_name: str):
    """
    Get a QoS profile by name

    Args:
        profile_name (str): Name of the profile to retrieve

    Returns:
        QoSProfile: The requested QoS profile
    """
    profiles = {
        'sensor_data': QoSProfiles.SENSOR_DATA,
        'joint_states': QoSProfiles.JOINT_STATES,
        'control_commands': QoSProfiles.CONTROL_COMMANDS,
        'emergency_commands': QoSProfiles.EMERGENCY_COMMANDS,
        'status_updates': QoSProfiles.STATUS_UPDATES,
        'parameters': QoSProfiles.PARAMETERS,
        'high_frequency_sensor': QoSProfiles.HIGH_FREQUENCY_SENSOR,
        'logging': QoSProfiles.LOGGING
    }

    if profile_name in profiles:
        return profiles[profile_name]
    else:
        # Return a default profile if the requested one doesn't exist
        return QoSProfiles.SENSOR_DATA


def create_custom_qos(
    history_depth: int = 10,
    reliability_reliable: bool = True,
    durability_volatile: bool = True
):
    """
    Create a custom QoS profile with specified parameters

    Args:
        history_depth (int): Number of messages to keep in history
        reliability_reliable (bool): Whether to use reliable (vs best-effort) delivery
        durability_volatile (bool): Whether to use volatile (vs transient-local) durability

    Returns:
        QoSProfile: Custom QoS profile
    """
    reliability = QoSReliabilityPolicy.RELIABLE if reliability_reliable else QoSReliabilityPolicy.BEST_EFFORT
    durability = QoSDurabilityPolicy.VOLATILE if durability_volatile else QoSDurabilityPolicy.TRANSIENT_LOCAL

    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=history_depth,
        reliability=reliability,
        durability=durability
    )


# Example usage and demonstration
if __name__ == "__main__":
    from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default

    print("QoS Profiles for Humanoid Robotics:")
    print(f"Joint States: {QoSProfiles.JOINT_STATES}")
    print(f"Control Commands: {QoSProfiles.CONTROL_COMMANDS}")
    print(f"Emergency Commands: {QoSProfiles.EMERGENCY_COMMANDS}")

    # Example of using a profile
    custom_profile = create_custom_qos(
        history_depth=5,
        reliability_reliable=True,
        durability_volatile=True
    )
    print(f"Custom Profile: {custom_profile}")