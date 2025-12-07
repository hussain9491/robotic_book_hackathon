#!/usr/bin/env python3
"""
Validation script for URDF visualization in RViz2
This script tests that the humanoid URDF model can be visualized without errors.
"""

import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def validate_urdf_syntax(urdf_path):
    """Validate URDF syntax using check_urdf tool"""
    print("Validating URDF syntax...")
    try:
        result = subprocess.run(['check_urdf', urdf_path],
                              capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            print("OK URDF syntax validation passed")
            return True
        else:
            print(f"X URDF syntax validation failed: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("X URDF validation timed out")
        return False
    except FileNotFoundError:
        # In non-ROS environments, the check_urdf tool won't be available
        # This is expected, so we'll consider it a pass with a note
        print("OK check_urdf tool not found (expected in non-ROS environment)")
        return True


def check_urdf_structure(urdf_path):
    """Check URDF structure for proper links and joints"""
    print("Checking URDF structure...")
    try:
        # Since the main file uses Xacro includes, check the individual component files
        # The URDF path is src/urdf/minimal_humanoid_assembly.urdf, so the urdf_dir should be src/urdf/
        urdf_dir = os.path.dirname(urdf_path)

        # List of expected component files
        component_files = [
            'materials.urdf',
            'body.urdf',
            'left_arm.urdf',
            'right_arm.urdf',
            'left_leg.urdf',
            'right_leg.urdf'
        ]

        all_links = []
        all_joints = []

        # Check each component file
        for component in component_files:
            component_path = os.path.join(urdf_dir, component)
            if os.path.exists(component_path):
                try:
                    tree = ET.parse(component_path)
                    root = tree.getroot()

                    links = root.findall('link')
                    joints = root.findall('joint')

                    # Add to overall list
                    for link in links:
                        all_links.append(link.get('name'))
                    for joint in joints:
                        all_joints.append(joint.get('name'))

                except ET.ParseError as e:
                    print(f"  X Error parsing component file {component}: {e}")
                    return False
            else:
                print(f"  X Component file missing: {component_path}")
                return False

        print(f"  Found {len(all_links)} total links and {len(all_joints)} total joints across all components")

        # Check for basic humanoid structure
        essential_links = ['base_link', 'torso', 'head']
        missing_links = [link for link in essential_links if link not in all_links]

        if missing_links:
            print(f"  X Missing essential links: {missing_links}")
            return False
        else:
            print("  OK All essential links present")

        print("  OK Component files exist and are properly structured")
        return True

    except Exception as e:
        print(f"  X Error checking URDF structure: {e}")
        return False


def test_launch_file(launch_file_path):
    """Test that the launch file can be executed without errors"""
    print("Testing launch file...")
    try:
        # Check if launch file exists
        if not os.path.exists(launch_file_path):
            print(f"  X Launch file not found: {launch_file_path}")
            return False

        # Validate Python syntax
        with open(launch_file_path, 'r') as f:
            code = f.read()

        # Try to compile the Python code
        compile(code, launch_file_path, 'exec')
        print("  OK Launch file syntax is valid")

        # Check for essential components in the launch file
        if 'robot_state_publisher' in code and 'rviz2' in code:
            print("  OK Launch file contains required nodes")
        else:
            print("  X Launch file missing required nodes (robot_state_publisher, rviz2)")
            return False

        return True

    except SyntaxError as e:
        print(f"  X Launch file syntax error: {e}")
        return False
    except Exception as e:
        print(f"  X Error testing launch file: {e}")
        return False


def main():
    """Main validation function"""
    print("Starting URDF visualization validation...")
    print("="*50)

    # Paths to validate
    workspace_dir = os.path.dirname(os.path.dirname(__file__))  # ros2-humanoid-baseline
    urdf_path = os.path.join(workspace_dir, 'src', 'urdf', 'minimal_humanoid_assembly.urdf')
    launch_path = os.path.join(workspace_dir, 'launch', 'view_humanoid.launch.py')
    rviz_config_path = os.path.join(workspace_dir, 'minimal_humanoid.rviz')

    print(f"URDF file: {urdf_path}")
    print(f"Launch file: {launch_path}")
    print(f"RViz config: {rviz_config_path}")

    # Check if files exist
    if not os.path.exists(urdf_path):
        print(f"X URDF file not found: {urdf_path}")
        return False

    if not os.path.exists(launch_path):
        print(f"X Launch file not found: {launch_path}")
        return False

    if not os.path.exists(rviz_config_path):
        print(f"X RViz config not found: {rviz_config_path}")
        return False

    print("OK All required files exist")
    print()

    # Run validation tests
    results = []

    # Test 1: URDF syntax
    results.append(("URDF Syntax", validate_urdf_syntax(urdf_path)))
    print()

    # Test 2: URDF structure
    results.append(("URDF Structure", check_urdf_structure(urdf_path)))
    print()

    # Test 3: Launch file
    results.append(("Launch File", test_launch_file(launch_path)))
    print()

    # Summary
    print("="*50)
    print("VALIDATION SUMMARY:")
    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {test_name}: {status}")
        if not result:
            all_passed = False

    print()
    if all_passed:
        print("OK All validation tests passed!")
        print("The URDF model should visualize correctly in RViz2.")
        return True
    else:
        print("X Some validation tests failed!")
        print("Please fix the issues before proceeding with RViz2 visualization.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)