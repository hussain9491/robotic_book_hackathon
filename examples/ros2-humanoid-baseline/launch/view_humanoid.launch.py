from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
import os


def generate_launch_description():
    # Define the URDF file path - using the original URDF file
    urdf_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'src',
        'minimal_humanoid.urdf'
    )

    # Define launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['cat ', urdf_file])  # Use 'cat' to read the URDF file
        }]
    )

    # Joint State Publisher node (GUI for testing joint movements)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',  # Use joint_state_publisher_gui for interactive testing
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            os.path.dirname(__file__),
            '..',
            'minimal_humanoid.rviz'
        )],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])