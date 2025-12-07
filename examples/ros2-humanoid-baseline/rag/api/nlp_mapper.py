"""
Natural Language to ROS 2 Command Mapper
This module implements the conversion of natural language commands to ROS 2 service calls
"""

import re
from typing import Dict, Any, Optional, Tuple
import logging

logger = logging.getLogger(__name__)

class NLPCommandMapper:
    """Converts natural language commands to ROS 2 service calls"""

    def __init__(self):
        # Define patterns for different types of commands
        self.patterns = {
            # Enable/disable commands
            "enable_control": [
                r"enable.*control",
                r"start.*control",
                r"activate.*control",
                r"turn.*on.*control"
            ],
            "disable_control": [
                r"disable.*control",
                r"stop.*control",
                r"deactivate.*control",
                r"turn.*off.*control",
                r"emergency.*stop",
                r"kill.*switch"
            ],

            # Joint movement commands
            "move_joint": [
                r"move.*(to|position|angle).*",
                r"set.*(to|position|angle).*",
                r"go.*(to|position|angle).*",
                r"rotate.*(to|position|angle).*",
                r"turn.*(to|position|angle).*"
            ],

            # Preset pose commands
            "preset_pose": [
                r"go.*to.*(ready|rest|attention|wave|sit|stand)",
                r"move.*to.*(ready|rest|attention|wave|sit|stand).*pose",
                r"take.*(ready|rest|attention|wave|sit|stand).*pose",
                r"set.*to.*(ready|rest|attention|wave|sit|stand).*position",
                r"(ready|rest|attention|wave|sit|stand).*pose",
                r"wave.*arm",  # Special case for waving
                r"go.*to.*(rest|attention)",  # Alternative patterns
                r"wave(?!.*position)"  # Wave command that's not about position
            ],

            # Query commands
            "get_state": [
                r"what.*state",
                r"current.*state",
                r"robot.*state",
                r"get.*state",
                r"tell.*position",
                r"where.*arm",
                r"where.*leg",
                r"what.*position"
            ],

            # Safety check commands
            "check_safety": [
                r"check.*safety",
                r"is.*safe",
                r"safety.*check",
                r"any.*violations",
                r"are.*limits.*ok"
            ]
        }

        # Joint name mappings (natural language to ROS joint names)
        self.joint_mappings = {
            # Head
            "head": "head_pan",
            "head pan": "head_pan",
            "neck": "head_pan",

            # Left arm
            "left shoulder": "left_shoulder_pitch",
            "left shoulder pitch": "left_shoulder_pitch",
            "left shoulder roll": "left_shoulder_roll",
            "left arm": "left_shoulder_pitch",  # Default to shoulder pitch
            "left elbow": "left_elbow_joint",
            "left elbow joint": "left_elbow_joint",

            # Right arm
            "right shoulder": "right_shoulder_pitch",
            "right shoulder pitch": "right_shoulder_pitch",
            "right shoulder roll": "right_shoulder_roll",
            "right arm": "right_shoulder_pitch",  # Default to shoulder pitch
            "right elbow": "right_elbow_joint",
            "right elbow joint": "right_elbow_joint",

            # Left leg
            "left hip": "left_hip_pitch",
            "left hip pitch": "left_hip_pitch",
            "left leg": "left_hip_pitch",  # Default to hip pitch
            "left knee": "left_knee_joint",
            "left knee joint": "left_knee_joint",

            # Right leg
            "right hip": "right_hip_pitch",
            "right hip pitch": "right_hip_pitch",
            "right leg": "right_hip_pitch",  # Default to hip pitch
            "right knee": "right_knee_joint",
            "right knee joint": "right_knee_joint",
        }

        # Position mappings (natural language to numeric values)
        self.position_mappings = {
            "zero": 0.0,
            "center": 0.0,
            "middle": 0.0,
            "straight": 0.0,
            "up": 0.5,
            "down": -0.5,
            "forward": 0.2,
            "backward": -0.2,
            "back": -0.2,
            "extended": -1.0,
            "bent": -0.5,
            "folded": -0.8,
            "raised": 0.5,
            "lowered": -0.3,
            "horizontal": 0.0,
            "vertical": 1.57,
        }

        # Pose mappings
        self.pose_mappings = {
            "ready": "ready",
            "rest": "rest",
            "attention": "attention",
            "wave": "wave",
            "sit": "sit",
            "stand": "stand"
        }

    def extract_number(self, text: str) -> Optional[float]:
        """Extract numeric values from text"""
        # Look for numbers in various formats
        number_pattern = r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?"
        matches = re.findall(number_pattern, text)
        if matches:
            try:
                return float(matches[0])
            except ValueError:
                pass
        return None

    def extract_joint_name(self, text: str) -> Optional[str]:
        """Extract joint name from text"""
        text_lower = text.lower()

        # Check for exact matches first
        for natural_name, ros_name in self.joint_mappings.items():
            if natural_name in text_lower:
                return ros_name

        # Check for partial matches
        for natural_name, ros_name in self.joint_mappings.items():
            if natural_name.split()[0] in text_lower:  # Check first word
                return ros_name

        return None

    def extract_position(self, text: str) -> Optional[float]:
        """Extract position value from text"""
        text_lower = text.lower()

        # Check for named positions
        for name, value in self.position_mappings.items():
            if name in text_lower:
                return value

        # Extract numeric value
        number = self.extract_number(text_lower)
        if number is not None:
            # Ensure the number is in a reasonable range for joint positions
            if -3.14 <= number <= 3.14:
                return number

        return None

    def extract_pose(self, text: str) -> Optional[str]:
        """Extract pose name from text"""
        text_lower = text.lower()

        for pose_name in self.pose_mappings:
            if pose_name in text_lower:
                return pose_name

        return None

    def identify_command_type(self, text: str) -> str:
        """Identify the type of command from natural language"""
        text_lower = text.lower()

        # Define priority order for command types
        # More specific types should be checked first
        priority_order = ["preset_pose", "enable_control", "disable_control",
                         "get_state", "check_safety", "move_joint"]

        for cmd_type in priority_order:
            if cmd_type in self.patterns:
                for pattern in self.patterns[cmd_type]:
                    if re.search(pattern, text_lower):
                        # Special handling to avoid "move to unknown position" matching preset_pose
                        if cmd_type == "preset_pose" and "unknown" in text_lower:
                            continue  # Skip preset pose for unknown position
                        return cmd_type

        # If no specific pattern matches, assume it's a move command
        return "move_joint"

    def map_to_ros_call(self, command: str) -> Optional[Dict[str, Any]]:
        """Map natural language command to ROS 2 service call"""
        logger.info(f"Mapping natural language command: {command}")

        cmd_type = self.identify_command_type(command)

        if cmd_type == "enable_control":
            return {
                "function_name": "enable_joint_control",
                "arguments": {}
            }
        elif cmd_type == "disable_control":
            return {
                "function_name": "disable_joint_control",
                "arguments": {}
            }
        elif cmd_type == "get_state":
            return {
                "function_name": "get_robot_state",
                "arguments": {}
            }
        elif cmd_type == "check_safety":
            return {
                "function_name": "check_safety_violations",
                "arguments": {}
            }
        elif cmd_type == "preset_pose":
            pose = self.extract_pose(command)
            if pose:
                return {
                    "function_name": "execute_preset_pose",
                    "arguments": {"pose_name": pose}
                }
            else:
                # If we can't identify a specific pose, return a default
                return {
                    "function_name": "execute_preset_pose",
                    "arguments": {"pose_name": "rest"}
                }
        elif cmd_type == "move_joint":
            # Extract joint name and position
            joint_name = self.extract_joint_name(command)
            position = self.extract_position(command)

            # If we can't extract position from named values, try to parse numbers
            if position is None:
                position = self.extract_number(command)

            # If we have both joint and position, create a single joint move
            if joint_name and position is not None:
                return {
                    "function_name": "move_joint_to_position",
                    "arguments": {
                        "joint_name": joint_name,
                        "position": position
                    }
                }
            # If we have a joint but no specific position, try to infer from context
            elif joint_name:
                # Look for directional words
                command_lower = command.lower()
                if "up" in command_lower or "raise" in command_lower:
                    position = 0.5
                elif "down" in command_lower or "lower" in command_lower:
                    position = -0.3
                elif "forward" in command_lower or "front" in command_lower:
                    position = 0.2
                elif "back" in command_lower or "backward" in command_lower:
                    position = -0.2
                elif "center" in command_lower or "middle" in command_lower or "zero" in command_lower:
                    position = 0.0
                else:
                    # Default position based on joint type
                    if "elbow" in joint_name or "knee" in joint_name:
                        position = -0.5  # Default bent position
                    else:
                        position = 0.0  # Default neutral position

                return {
                    "function_name": "move_joint_to_position",
                    "arguments": {
                        "joint_name": joint_name,
                        "position": position
                    }
                }
            else:
                # If we can't identify specific joints, try to interpret as a general movement
                # This might be a more complex command that requires context
                return None

        return None

    def parse_complex_command(self, command: str) -> Optional[Dict[str, Any]]:
        """Parse more complex commands that might involve multiple joints or poses"""
        command_lower = command.lower()

        # Look for pose commands that might not be caught by the regex patterns
        for pose_name in self.pose_mappings:
            if pose_name in command_lower:
                # Check if it's a pose command
                pose_indicators = [f"to {pose_name}", f"to a {pose_name}", f"to the {pose_name}",
                                  f"{pose_name} pose", f"set.*{pose_name}", f"go.*{pose_name}",
                                  f"move.*{pose_name}", f"take.*{pose_name}"]

                for indicator in pose_indicators:
                    if re.search(indicator, command_lower):
                        return {
                            "function_name": "execute_preset_pose",
                            "arguments": {"pose_name": pose_name}
                        }

        # Look for multi-joint commands like "raise both arms" or "move left arm up and right arm down"
        multi_joint_indicators = ["both", "all", "two", "and", "move.*left.*and.*right"]

        for indicator in multi_joint_indicators:
            if re.search(indicator, command_lower):
                # This is a simplified implementation
                # In a real system, you'd need more sophisticated NLP
                if "both arms" in command_lower or "two arms" in command_lower:
                    # Determine positions based on the rest of the command
                    if "up" in command_lower or "raise" in command_lower:
                        return {
                            "function_name": "move_multiple_joints",
                            "arguments": {
                                "joints": [
                                    {"joint_name": "left_shoulder_pitch", "position": 0.5},
                                    {"joint_name": "right_shoulder_pitch", "position": 0.5},
                                    {"joint_name": "left_elbow_joint", "position": -0.5},
                                    {"joint_name": "right_elbow_joint", "position": -0.5}
                                ]
                            }
                        }
                    elif "down" in command_lower or "lower" in command_lower:
                        return {
                            "function_name": "move_multiple_joints",
                            "arguments": {
                                "joints": [
                                    {"joint_name": "left_shoulder_pitch", "position": -0.2},
                                    {"joint_name": "right_shoulder_pitch", "position": -0.2},
                                    {"joint_name": "left_elbow_joint", "position": 0.0},
                                    {"joint_name": "right_elbow_joint", "position": 0.0}
                                ]
                            }
                        }

        return None

    def map_command(self, command: str) -> Optional[Dict[str, Any]]:
        """Main method to map a natural language command to a ROS 2 call"""
        # First try to parse as a complex command
        result = self.parse_complex_command(command)
        if result:
            return result

        # Then try simple mapping
        result = self.map_to_ros_call(command)
        if result:
            return result

        # If no mapping found, return None
        logger.warning(f"Could not map command to ROS call: {command}")
        return None


# Example usage and testing
def test_nlp_mapper():
    """Test the NLP mapper with sample commands"""
    mapper = NLPCommandMapper()

    test_commands = [
        "Enable robot control",
        "Disable robot control",
        "Move left shoulder to 0.5 radians",
        "Set right elbow to -0.3 position",
        "Go to ready pose",
        "What is the current robot state?",
        "Check safety violations",
        "Raise left arm",
        "Lower right arm",
        "Move head to center",
        "Both arms up"
    ]

    print("Testing NLP Command Mapper:")
    print("=" * 50)

    for cmd in test_commands:
        result = mapper.map_command(cmd)
        print(f"Command: '{cmd}'")
        print(f"Result: {result}")
        print("-" * 30)


if __name__ == "__main__":
    test_nlp_mapper()