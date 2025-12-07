"""
Configuration management system for ROS 2 parameters
Used in the Physical AI & Humanoid Robotics educational examples
"""

import json
import yaml
from pathlib import Path
from typing import Any, Dict, Optional


class ConfigManager:
    """
    A configuration management system for ROS 2 parameters
    Allows loading, saving, and managing configuration parameters
    """

    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize the configuration manager

        Args:
            config_file (str, optional): Path to the config file to load
        """
        self.config = {}
        if config_file and Path(config_file).exists():
            self.load_config(config_file)

    def load_config(self, config_file: str):
        """
        Load configuration from a file (JSON or YAML)

        Args:
            config_file (str): Path to the configuration file
        """
        config_path = Path(config_file)

        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file {config_file} does not exist")

        with open(config_path, 'r') as f:
            if config_path.suffix.lower() in ['.yaml', '.yml']:
                self.config = yaml.safe_load(f)
            elif config_path.suffix.lower() == '.json':
                self.config = json.load(f)
            else:
                raise ValueError(f"Unsupported config file format: {config_path.suffix}")

    def save_config(self, config_file: str):
        """
        Save configuration to a file (JSON or YAML based on extension)

        Args:
            config_file (str): Path to save the configuration file
        """
        config_path = Path(config_file)

        with open(config_path, 'w') as f:
            if config_path.suffix.lower() in ['.yaml', '.yml']:
                yaml.dump(self.config, f, default_flow_style=False)
            elif config_path.suffix.lower() == '.json':
                json.dump(self.config, f, indent=2)
            else:
                raise ValueError(f"Unsupported config file format: {config_path.suffix}")

    def set_param(self, key: str, value: Any):
        """
        Set a configuration parameter

        Args:
            key (str): Parameter key
            value (Any): Parameter value
        """
        self.config[key] = value

    def get_param(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration parameter

        Args:
            key (str): Parameter key
            default (Any): Default value if key doesn't exist

        Returns:
            Any: Parameter value or default
        """
        return self.config.get(key, default)

    def get_nested_param(self, key_path: str, delimiter: str = '.', default: Any = None) -> Any:
        """
        Get a nested configuration parameter using dot notation

        Args:
            key_path (str): Parameter key path using delimiter (e.g. 'robot.arm.joint_limits')
            delimiter (str): Delimiter for nested keys
            default (Any): Default value if path doesn't exist

        Returns:
            Any: Parameter value or default
        """
        keys = key_path.split(delimiter)
        current = self.config

        try:
            for key in keys:
                current = current[key]
            return current
        except (KeyError, TypeError):
            return default

    def update_params(self, params: Dict[str, Any]):
        """
        Update multiple parameters at once

        Args:
            params (Dict[str, Any]): Dictionary of parameters to update
        """
        self.config.update(params)

    def get_all_params(self) -> Dict[str, Any]:
        """
        Get all configuration parameters

        Returns:
            Dict[str, Any]: All configuration parameters
        """
        return self.config.copy()

    def to_ros_params(self) -> Dict[str, Any]:
        """
        Convert configuration to ROS 2 parameter format

        Returns:
            Dict[str, Any]: Parameters in ROS 2 compatible format
        """
        # Convert nested dictionaries to ROS 2 parameter names with underscores
        ros_params = {}

        def flatten_dict(d, parent_key='', sep='_'):
            items = []
            for k, v in d.items():
                new_key = f"{parent_key}{sep}{k}" if parent_key else k
                if isinstance(v, dict):
                    items.extend(flatten_dict(v, new_key, sep=sep).items())
                else:
                    items.append((new_key, v))
            return dict(items)

        return flatten_dict(self.config)


# Example usage
if __name__ == "__main__":
    # Create a sample configuration
    config = ConfigManager()
    config.set_param('robot_name', 'humanoid_robot')
    config.set_param('control_loop_rate', 50)  # Hz

    # Nested parameters
    config.update_params({
        'joints': {
            'head': {'min': -1.57, 'max': 1.57},
            'arm': {'min': -2.0, 'max': 2.0},
            'leg': {'min': -1.0, 'max': 1.0}
        },
        'safety': {
            'max_velocity': 1.0,
            'max_torque': 100.0
        }
    })

    # Save to file
    config.save_config('robot_config.json')
    print("Configuration saved to robot_config.json")
    print("Full config:", config.get_all_params())
    print("Joint head max:", config.get_nested_param('joints.head.max'))