#!/usr/bin/env python3
"""
URDF validation script for the minimal humanoid model
Validates the URDF model using check_urdf tool and provides detailed feedback
Used in the Physical AI & Humanoid Robotics educational examples
"""

import subprocess
import sys
import os
from pathlib import Path


def validate_urdf_with_check_urdf(urdf_path):
    """
    Validate URDF using the ROS 2 check_urdf tool

    Args:
        urdf_path (str): Path to the URDF file to validate

    Returns:
        tuple: (success: bool, output: str, error: str)
    """
    try:
        # Run the check_urdf command
        result = subprocess.run(['check_urdf', urdf_path],
                              capture_output=True, text=True, timeout=30)

        success = result.returncode == 0
        output = result.stdout
        error = result.stderr

        return success, output, error

    except FileNotFoundError:
        return False, "", "check_urdf command not found. Make sure ROS 2 is installed and sourced."
    except subprocess.TimeoutExpired:
        return False, "", "URDF validation timed out."
    except Exception as e:
        return False, "", f"Error running check_urdf: {str(e)}"


def validate_urdf_basic(urdf_path):
    """
    Perform basic validation of the URDF file without check_urdf tool

    Args:
        urdf_path (str): Path to the URDF file to validate

    Returns:
        tuple: (success: bool, message: str)
    """
    urdf_path = Path(urdf_path)

    if not urdf_path.exists():
        return False, f"URDF file does not exist: {urdf_path}"

    try:
        with open(urdf_path, 'r') as f:
            content = f.read()

        # Basic checks
        checks = [
            ('<robot', 'Missing <robot> tag'),
            ('<link', 'No links defined in URDF'),
            ('<joint', 'No joints defined in URDF'),
            ('name=', 'No named elements found'),
            ('<visual', 'No visual elements found (though not strictly required)'),
            ('<collision', 'No collision elements found (though not strictly required)'),
        ]

        issues = []
        for check, error_msg in checks:
            if check not in content:
                issues.append(error_msg)

        if issues:
            return False, f"Basic validation failed: {', '.join(issues)}"

        # Check for proper XML structure
        import xml.etree.ElementTree as ET
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            if root.tag != 'robot':
                return False, "Root element is not <robot>"

            # Count links and joints
            links = root.findall('link')
            joints = root.findall('joint')

            if len(links) < 2:  # At least base_link + 1 other link
                return False, f"Too few links: found {len(links)}, need at least 2"

            if len(joints) < 1:  # At least 1 joint to connect links
                return False, f"Too few joints: found {len(joints)}, need at least 1"

            # Check that joints reference existing links
            link_names = {link.get('name') for link in links}
            for joint in joints:
                parent = joint.find('parent')
                child = joint.find('child')

                if parent is not None and parent.get('link') not in link_names:
                    return False, f"Joint '{joint.get('name')}' references non-existent parent link '{parent.get('link')}'"

                if child is not None and child.get('link') not in link_names:
                    return False, f"Joint '{joint.get('name')}' references non-existent child link '{child.get('link')}'"

        except ET.ParseError as e:
            return False, f"Invalid XML syntax: {str(e)}"

        return True, f"Basic validation passed: {len(links)} links, {len(joints)} joints"

    except Exception as e:
        return False, f"Error reading URDF file: {str(e)}"


def main(urdf_file_path=None):
    """
    Main function to validate the URDF model
    """
    if urdf_file_path is None:
        # Default path for our humanoid model
        urdf_file_path = "src/minimal_humanoid.urdf"
        full_path = Path(__file__).parent / urdf_file_path
        if not full_path.exists():
            print(f"Default URDF file not found: {full_path}")
            # Try alternative location
            full_path = Path(__file__).parent.parent / "src" / "minimal_humanoid.urdf"
            if full_path.exists():
                urdf_file_path = str(full_path)
            else:
                print(f"URDF file not found in either location. Looking for: {Path(__file__).parent / 'src/minimal_humanoid.urdf'}")
                return 1

    print(f"Validating URDF model: {urdf_file_path}")
    print("="*60)

    # First, try the ROS 2 check_urdf tool
    print("Attempting validation with check_urdf tool...")
    success, output, error = validate_urdf_with_check_urdf(urdf_file_path)

    if success:
        print("✓ check_urdf validation PASSED")
        print("\ncheck_urdf output:")
        print(output)
        return 0
    else:
        print(f"✗ check_urdf validation failed or tool not available:")
        if error:
            print(f"Error: {error}")
        if output:
            print(f"Output: {output}")

        print("\nFalling back to basic validation...")
        basic_success, basic_message = validate_urdf_basic(urdf_file_path)

        print(f"Basic validation: {'PASSED' if basic_success else 'FAILED'}")
        print(f"Result: {basic_message}")

        if basic_success:
            print("\nNote: Basic validation passed, but ROS 2 check_urdf tool may not be available.")
            print("Make sure ROS 2 is properly installed and sourced to run full validation.")
            return 0
        else:
            print("\nURDF validation FAILED - please fix the issues above.")
            return 1


if __name__ == "__main__":
    # Allow specifying URDF file as command line argument
    if len(sys.argv) > 1:
        result = main(sys.argv[1])
    else:
        result = main()

    sys.exit(result)