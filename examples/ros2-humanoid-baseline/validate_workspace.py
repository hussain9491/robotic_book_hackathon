#!/usr/bin/env python3
"""
Validation scripts for URDF and ROS 2 workspace testing
Used in the Physical AI & Humanoid Robotics educational examples
"""

import os
import sys
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple


def validate_urdf_file(urdf_path: str) -> Tuple[bool, str]:
    """
    Validate a URDF file using check_urdf command (if available)

    Args:
        urdf_path (str): Path to the URDF file to validate

    Returns:
        Tuple[bool, str]: (is_valid, message)
    """
    urdf_path = Path(urdf_path)

    if not urdf_path.exists():
        return False, f"URDF file does not exist: {urdf_path}"

    # Check if check_urdf command is available
    try:
        result = subprocess.run(['which', 'check_urdf'],
                              capture_output=True, text=True)
        if result.returncode != 0:
            return False, "check_urdf command not found. Install ROS 2 urdf package."

        # Run check_urdf on the file
        result = subprocess.run(['check_urdf', str(urdf_path)],
                              capture_output=True, text=True)

        if result.returncode == 0:
            return True, "URDF validation passed"
        else:
            return False, f"URDF validation failed: {result.stderr}"

    except FileNotFoundError:
        # Fallback: basic XML validation
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            if root.tag != 'robot':
                return False, "Root element is not 'robot'"

            # Basic checks for required elements
            links = root.findall('link')
            joints = root.findall('joint')

            if len(links) == 0:
                return False, "No links found in URDF"

            if len(joints) == 0:
                # This might be okay for a simple single-link robot
                pass

            return True, "Basic XML structure validation passed (install ROS 2 for full validation)"

        except ET.ParseError as e:
            return False, f"Invalid XML syntax: {str(e)}"


def validate_ros_workspace(workspace_path: str) -> Tuple[bool, str]:
    """
    Validate basic ROS 2 workspace structure

    Args:
        workspace_path (str): Path to the ROS 2 workspace

    Returns:
        Tuple[bool, str]: (is_valid, message)
    """
    workspace_path = Path(workspace_path)

    if not workspace_path.exists():
        return False, f"Workspace directory does not exist: {workspace_path}"

    required_dirs = ['src', 'build', 'install']
    missing_dirs = []

    for directory in required_dirs:
        dir_path = workspace_path / directory
        if not dir_path.exists():
            missing_dirs.append(str(dir_path))

    if missing_dirs:
        return False, f"Missing required directories: {', '.join(missing_dirs)}"

    # Check if src directory has any packages
    src_path = workspace_path / 'src'
    if not any(src_path.iterdir()):
        return False, "src directory is empty - no packages found"

    return True, "Workspace structure validation passed"


def validate_python_environment() -> Tuple[bool, str]:
    """
    Validate Python environment for ROS 2 development

    Returns:
        Tuple[bool, str]: (is_valid, message)
    """
    try:
        import rclpy
        return True, "rclpy is available in Python environment"
    except ImportError:
        return False, "rclpy not available - ROS 2 Python packages not installed"


def run_all_validations(workspace_path: str = None, urdf_path: str = None) -> List[Tuple[str, bool, str]]:
    """
    Run all validation checks

    Args:
        workspace_path (str, optional): Path to workspace to validate
        urdf_path (str, optional): Path to URDF file to validate

    Returns:
        List[Tuple[str, bool, str]]: List of (test_name, is_passed, message)
    """
    results = []

    # Validate Python environment
    env_ok, env_msg = validate_python_environment()
    results.append(("Python Environment", env_ok, env_msg))

    # Validate workspace if path provided
    if workspace_path:
        ws_ok, ws_msg = validate_ros_workspace(workspace_path)
        results.append(("ROS Workspace Structure", ws_ok, ws_msg))

    # Validate URDF if path provided
    if urdf_path:
        urdf_ok, urdf_msg = validate_urdf_file(urdf_path)
        results.append(("URDF File", urdf_ok, urdf_msg))

    return results


def print_validation_report(results: List[Tuple[str, bool, str]]):
    """
    Print a formatted validation report

    Args:
        results: List of validation results
    """
    print("=" * 60)
    print("VALIDATION REPORT")
    print("=" * 60)

    all_passed = True

    for test_name, is_passed, message in results:
        status = "PASS" if is_passed else "FAIL"
        status_icon = "✓" if is_passed else "✗"
        print(f"{status_icon} {test_name:<30} [{status}]")
        print(f"  {message}")
        print()

        if not is_passed:
            all_passed = False

    print("=" * 60)
    overall_status = "PASS" if all_passed else "FAIL"
    print(f"OVERALL: {overall_status}")
    print("=" * 60)


if __name__ == "__main__":
    # Default paths for testing
    workspace_path = "examples/ros2-humanoid-baseline"
    urdf_path = "examples/ros2-humanoid-baseline/src/simple_robot.urdf"  # This may not exist yet

    print("Running validation checks...")
    results = run_all_validations(workspace_path, urdf_path)
    print_validation_report(results)

    # Exit with error code if any validation failed
    if not all(result[1] for result in results):
        sys.exit(1)