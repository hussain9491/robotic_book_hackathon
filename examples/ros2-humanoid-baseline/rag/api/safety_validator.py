"""
Safety validation layer for the ROS 2 AI Agent Bridge
This module implements safety checks for position/velocity limits and other constraints
"""

from typing import Dict, Any, List, Tuple
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class SafetyValidator:
    """Safety validation layer for robot control commands"""

    def __init__(self):
        # Define joint limits for the humanoid robot
        self.joint_limits = {
            "head_pan": {"min": -1.57, "max": 1.57, "velocity": 2.0, "effort": 100.0},
            "left_shoulder_pitch": {"min": -2.0, "max": 2.0, "velocity": 2.0, "effort": 100.0},
            "left_shoulder_roll": {"min": -1.57, "max": 1.57, "velocity": 2.0, "effort": 100.0},
            "left_elbow_joint": {"min": -2.0, "max": 0.0, "velocity": 2.0, "effort": 100.0},
            "right_shoulder_pitch": {"min": -2.0, "max": 2.0, "velocity": 2.0, "effort": 100.0},
            "right_shoulder_roll": {"min": -1.57, "max": 1.57, "velocity": 2.0, "effort": 100.0},
            "right_elbow_joint": {"min": -2.0, "max": 0.0, "velocity": 2.0, "effort": 100.0},
            "left_hip_pitch": {"min": -1.0, "max": 1.0, "velocity": 1.5, "effort": 200.0},
            "left_knee_joint": {"min": 0.0, "max": 2.0, "velocity": 1.5, "effort": 200.0},
            "right_hip_pitch": {"min": -1.0, "max": 1.0, "velocity": 1.5, "effort": 200.0},
            "right_knee_joint": {"min": 0.0, "max": 2.0, "velocity": 1.5, "effort": 200.0}
        }

        # Define safety thresholds
        self.max_joint_velocity = 5.0  # rad/s
        self.max_joint_effort = 300.0  # Nm
        self.max_position_change = 1.0  # rad per command (for smooth motion)

        # Current joint states (would be updated from robot feedback)
        self.current_states = {
            "head_pan": 0.0,
            "left_shoulder_pitch": 0.2,
            "left_shoulder_roll": 0.1,
            "left_elbow_joint": -0.5,
            "right_shoulder_pitch": 0.2,
            "right_shoulder_roll": -0.1,
            "right_elbow_joint": -0.5,
            "left_hip_pitch": 0.0,
            "left_knee_joint": 0.0,
            "right_hip_pitch": 0.0,
            "right_knee_joint": 0.0
        }

    def validate_single_joint_command(self, joint_name: str, target_position: float) -> Tuple[bool, str]:
        """
        Validate a single joint command against safety limits
        """
        if joint_name not in self.joint_limits:
            violation_msg = f"Joint {joint_name} not found in safety database"
            logger.warning(f"Safety violation - {violation_msg}")
            return False, violation_msg

        limits = self.joint_limits[joint_name]

        # Check position limits
        if target_position < limits["min"] or target_position > limits["max"]:
            violation_msg = f"Position {target_position} for {joint_name} exceeds limits [{limits['min']}, {limits['max']}]"
            logger.warning(f"Safety violation - {violation_msg}")
            return False, violation_msg

        # Check velocity limits (if we have current state)
        if joint_name in self.current_states:
            current_pos = self.current_states[joint_name]
            pos_change = abs(target_position - current_pos)

            if pos_change > self.max_position_change:
                violation_msg = f"Position change {pos_change} for {joint_name} exceeds max change {self.max_position_change}"
                logger.warning(f"Safety violation - {violation_msg}")
                return False, violation_msg

        return True, "Command is safe"

    def validate_multiple_joint_commands(self, joints_data: List[Dict[str, Any]]) -> Tuple[bool, str, List[str]]:
        """
        Validate multiple joint commands against safety limits
        Returns: (is_safe, message, list_of_violations)
        """
        violations = []

        for joint_cmd in joints_data:
            joint_name = joint_cmd.get("joint_name")
            target_position = joint_cmd.get("position")

            if not joint_name or target_position is None:
                invalid_msg = "Missing joint_name or position in command"
                violations.append(invalid_msg)
                logger.warning(f"Invalid command - {invalid_msg}")
                continue

            is_safe, message = self.validate_single_joint_command(joint_name, target_position)
            if not is_safe:
                violations.append(f"{joint_name}: {message}")

        if violations:
            logger.warning(f"Multiple joint validation failed - {len(violations)} safety violations: {violations[:3]}")  # Log first 3 violations
            return False, f"Found {len(violations)} safety violations", violations
        else:
            logger.info("Multiple joint validation passed - all commands are safe")
            return True, "All commands are safe", []

    def validate_pose_command(self, pose_name: str, pose_joints: Dict[str, float]) -> Tuple[bool, str, List[str]]:
        """
        Validate a predefined pose command
        """
        violations = []

        for joint_name, target_position in pose_joints.items():
            is_safe, message = self.validate_single_joint_command(joint_name, target_position)
            if not is_safe:
                violations.append(f"{joint_name}: {message}")

        if violations:
            logger.warning(f"Pose validation failed for {pose_name} - {len(violations)} safety violations: {violations[:3]}")
            return False, f"Pose {pose_name} has safety violations", violations
        else:
            logger.info(f"Pose validation passed for {pose_name} - all commands are safe")
            return True, f"Pose {pose_name} is safe", []

    def update_current_state(self, joint_states: Dict[str, float]):
        """
        Update the current joint states from robot feedback
        """
        for joint_name, position in joint_states.items():
            if joint_name in self.current_states:
                self.current_states[joint_name] = position

    def check_safety_status(self) -> Dict[str, Any]:
        """
        Check overall safety status of the robot
        """
        # Check current positions against limits
        violations = []

        for joint_name, current_pos in self.current_states.items():
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                if current_pos < limits["min"] or current_pos > limits["max"]:
                    violations.append(f"Joint {joint_name} at {current_pos}, limits [{limits['min']}, {limits['max']}]")

        return {
            "safety_ok": len(violations) == 0,
            "violations": violations,
            "joint_limits": self.joint_limits,
            "current_states": self.current_states
        }

    def validate_command_sequence(self, commands: List[Dict[str, Any]]) -> Tuple[bool, str, List[str]]:
        """
        Validate a sequence of commands to ensure safety across the sequence
        """
        violations = []

        for i, command in enumerate(commands):
            cmd_type = command.get("type", "unknown")

            if cmd_type == "single_joint":
                joint_name = command.get("joint_name")
                target_position = command.get("position")
                if joint_name and target_position is not None:
                    is_safe, message = self.validate_single_joint_command(joint_name, target_position)
                    if not is_safe:
                        violations.append(f"Command {i}: {message}")
                else:
                    invalid_msg = f"Command {i}: Missing joint_name or position in single_joint command"
                    violations.append(invalid_msg)
                    logger.warning(f"Invalid command - {invalid_msg}")
            elif cmd_type == "multi_joint":
                joints_data = command.get("joints", [])
                if joints_data:
                    _, _, cmd_violations = self.validate_multiple_joint_commands(joints_data)
                    for violation in cmd_violations:
                        violations.append(f"Command {i}: {violation}")
                else:
                    invalid_msg = f"Command {i}: Missing joints data in multi_joint command"
                    violations.append(invalid_msg)
                    logger.warning(f"Invalid command - {invalid_msg}")
            elif cmd_type == "pose":
                pose_name = command.get("pose_name", "unknown")
                # For pose commands, we'd need access to predefined poses
                # This is a simplified check - in real implementation, you'd validate against actual pose data
                logger.info(f"Pose command {i} for {pose_name} - validation would require pose data")
            else:
                invalid_msg = f"Command {i}: Unknown command type '{cmd_type}'"
                violations.append(invalid_msg)
                logger.warning(f"Invalid command - {invalid_msg}")

        if violations:
            logger.warning(f"Command sequence validation failed - {len(violations)} safety violations: {violations[:5]}")  # Log first 5 violations
            return False, f"Command sequence has {len(violations)} safety violations", violations
        else:
            logger.info("Command sequence validation passed - all commands in sequence are safe")
            return True, "Command sequence is safe", []