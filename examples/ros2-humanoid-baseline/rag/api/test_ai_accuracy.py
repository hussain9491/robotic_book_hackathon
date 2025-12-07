#!/usr/bin/env python3
"""
Test script for AI agent accuracy in the ROS 2 AI Agent Bridge
This script tests the accuracy of natural language to ROS 2 command mapping
Target: 90% conversion accuracy
"""

import sys
import os
from typing import Dict, List, Tuple

# Add the api directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '.'))

from nlp_mapper import NLPCommandMapper
from function_definitions import PREDEFINED_POSES

def test_nlp_accuracy():
    """Test the accuracy of the NLP command mapper"""

    # Define test cases with expected outcomes
    test_cases = [
        # Enable/disable commands
        ("Enable robot control", "enable_joint_control", {}),
        ("Disable joint control", "disable_joint_control", {}),
        ("Turn off control", "disable_joint_control", {}),

        # Joint movement commands
        ("Move left shoulder to 0.5", "move_joint_to_position", {"joint_name": "left_shoulder_pitch", "position": 0.5}),
        ("Set right elbow to -0.3", "move_joint_to_position", {"joint_name": "right_elbow_joint", "position": -0.3}),
        ("Raise left arm", "move_joint_to_position", {"joint_name": "left_shoulder_pitch", "position": 0.5}),
        ("Lower right arm", "move_joint_to_position", {"joint_name": "right_shoulder_pitch", "position": -0.2}),
        ("Move head to center", "move_joint_to_position", {"joint_name": "head_pan", "position": 0.0}),

        # Multiple joint commands
        ("Both arms up", "move_multiple_joints", None),  # This should be a multi-joint command

        # Preset pose commands
        ("Go to ready pose", "execute_preset_pose", {"pose_name": "ready"}),
        ("Move to rest position", "execute_preset_pose", {"pose_name": "rest"}),
        ("Take attention pose", "execute_preset_pose", {"pose_name": "attention"}),
        ("Wave", "execute_preset_pose", {"pose_name": "wave"}),

        # Query commands
        ("What is the current robot state?", "get_robot_state", {}),
        ("Tell me the joint positions", "get_robot_state", {}),

        # Safety commands
        ("Check safety violations", "check_safety_violations", {}),
        ("Are the limits ok?", "check_safety_violations", {}),
    ]

    # Edge cases and invalid commands
    edge_cases = [
        ("Random invalid command", None, None),
        ("Move to unknown position", None, None),
        ("", None, None),  # Empty command
    ]

    mapper = NLPCommandMapper()
    total_tests = len(test_cases) + len(edge_cases)
    successful_mappings = 0
    detailed_results = []

    print("Testing NLP Command Mapping Accuracy")
    print("=" * 60)

    # Test regular cases
    for i, (command, expected_function, expected_args) in enumerate(test_cases):
        print(f"Test {i+1}: '{command}'")

        result = mapper.map_command(command)

        if result and result.get("function_name") == expected_function:
            if expected_args is None or expected_function == "move_multiple_joints":
                # For multi-joint commands or cases where we don't check specific args
                successful_mappings += 1
                status = "PASS"
            elif result.get("arguments") == expected_args:
                successful_mappings += 1
                status = "PASS"
            else:
                status = f"PARTIAL (expected {expected_args}, got {result.get('arguments')})"
        else:
            status = f"FAIL (expected {expected_function}, got {result})"

        detailed_results.append((command, result, status))
        print(f"  Result: {status}")
        print()

    # Test edge cases (these should mostly return None, which is acceptable)
    for i, (command, expected_function, expected_args) in enumerate(edge_cases):
        print(f"Edge Case {i+1}: '{command}'")

        result = mapper.map_command(command)

        # For invalid commands, it's acceptable to return None
        if result is None:
            successful_mappings += 1  # Count as success since we expect None for invalid commands
            status = "PASS (correctly rejected)"
        else:
            status = f"UNEXPECTED (got {result})"

        detailed_results.append((command, result, status))
        print(f"  Result: {status}")
        print()

    accuracy = (successful_mappings / total_tests) * 100
    target_accuracy = 90.0

    print("=" * 60)
    print(f"Test Results Summary:")
    print(f"Total Tests: {total_tests}")
    print(f"Successful Mappings: {successful_mappings}")
    print(f"Accuracy: {accuracy:.1f}%")
    print(f"Target: {target_accuracy}%")
    print(f"Status: {'PASS' if accuracy >= target_accuracy else 'FAIL'}")

    if accuracy >= target_accuracy:
        print("\n[SUCCESS] AI agent accuracy target achieved!")
        return True
    else:
        print(f"\n[FAILURE] AI agent accuracy target NOT met. Need to improve NLP mapping.")
        return False

def test_safety_validation():
    """Test the safety validation accuracy"""
    from safety_validator import SafetyValidator

    validator = SafetyValidator()

    print("\nTesting Safety Validation")
    print("=" * 40)

    # Test valid commands
    valid_tests = [
        ("left_shoulder_pitch", 0.5),
        ("right_elbow_joint", -0.5),
        ("head_pan", 0.0),
    ]

    # Test invalid commands (should be rejected)
    invalid_tests = [
        ("left_shoulder_pitch", 5.0),  # Exceeds limits
        ("right_elbow_joint", -3.0),  # Exceeds limits
        ("nonexistent_joint", 0.5),   # Joint doesn't exist
    ]

    valid_successes = 0
    for joint, pos in valid_tests:
        is_safe, msg = validator.validate_single_joint_command(joint, pos)
        if is_safe:
            valid_successes += 1
            print(f"[PASS] Valid command '{joint}' to {pos}: PASS")
        else:
            print(f"[FAIL] Valid command '{joint}' to {pos}: FAIL - {msg}")

    invalid_successes = 0
    for joint, pos in invalid_tests:
        is_safe, msg = validator.validate_single_joint_command(joint, pos)
        if not is_safe:
            invalid_successes += 1
            print(f"[PASS] Invalid command '{joint}' to {pos}: Correctly rejected")
        else:
            print(f"[FAIL] Invalid command '{joint}' to {pos}: Should have been rejected")

    total_valid_tests = len(valid_tests)
    total_invalid_tests = len(invalid_tests)

    valid_accuracy = (valid_successes / total_valid_tests) * 100 if total_valid_tests > 0 else 100
    invalid_accuracy = (invalid_successes / total_invalid_tests) * 100 if total_invalid_tests > 0 else 100

    print(f"\nSafety Validation Results:")
    print(f"Valid commands accepted: {valid_accuracy:.1f}% ({valid_successes}/{total_valid_tests})")
    print(f"Invalid commands rejected: {invalid_accuracy:.1f}% ({invalid_successes}/{total_invalid_tests})")

def main():
    """Main test function"""
    print("ROS 2 AI Agent Bridge - Accuracy Testing")
    print("=" * 60)

    # Test NLP accuracy
    nlp_success = test_nlp_accuracy()

    # Test safety validation
    test_safety_validation()

    print("\n" + "=" * 60)
    if nlp_success:
        print("[SUCCESS] All tests passed! AI agent accuracy meets target.")
        return 0
    else:
        print("[WARNING] Some tests failed. AI agent accuracy needs improvement.")
        return 1

if __name__ == "__main__":
    sys.exit(main())