#!/usr/bin/env python3
"""
Test script for the ROS 2 AI Agent Bridge server
"""

import requests
import json
import time
import subprocess
import sys
from typing import Dict, Any

# Test configuration
BASE_URL = "http://localhost:8000"

def test_health_endpoint():
    """Test the health endpoint"""
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/health")
        if response.status_code == 200:
            print("✓ Health endpoint: OK")
            return True
        else:
            print(f"✗ Health endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Health endpoint error: {e}")
        return False

def test_available_services():
    """Test the available services endpoint"""
    print("Testing available services endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/available-services")
        if response.status_code == 200:
            data = response.json()
            if "services" in data and len(data["services"]) > 0:
                print(f"✓ Available services: {len(data['services'])} services found")
                print(f"  Services: {data['services']}")
                return True
            else:
                print("✗ Available services endpoint returned empty service list")
                return False
        else:
            print(f"✗ Available services endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Available services endpoint error: {e}")
        return False

def test_natural_language_translation():
    """Test natural language command translation"""
    print("Testing natural language translation...")
    try:
        # Test enabling joint control
        command_data = {
            "command": "Enable joint control on the robot",
            "parameters": {},
            "robot_id": "test_robot"
        }

        response = requests.post(f"{BASE_URL}/translate-command",
                                json=command_data)
        if response.status_code == 200:
            result = response.json()
            if result["success"]:
                print("✓ Natural language translation: OK")
                print(f"  Response: {result['message']}")
                return True
            else:
                print(f"✗ Natural language translation failed: {result.get('error', 'Unknown error')}")
                return False
        else:
            print(f"✗ Natural language translation endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Natural language translation error: {e}")
        return False

def test_direct_service_call():
    """Test direct service call"""
    print("Testing direct service call...")
    try:
        service_data = {
            "service_name": "/robot_state/get",
            "service_type": "std_srvs/Trigger",
            "request_data": {}
        }

        response = requests.post(f"{BASE_URL}/call-service",
                                json=service_data)
        if response.status_code == 200:
            result = response.json()
            if result["success"]:
                print("✓ Direct service call: OK")
                print(f"  Response: {result['message']}")
                return True
            else:
                print(f"✗ Direct service call failed: {result.get('error', 'Unknown error')}")
                return False
        else:
            print(f"✗ Direct service call endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Direct service call error: {e}")
        return False

def test_function_call():
    """Test OpenAI function calling interface"""
    print("Testing function call interface...")
    try:
        function_data = {
            "name": "get_robot_state",
            "arguments": {}
        }

        response = requests.post(f"{BASE_URL}/function-call",
                                json=function_data)
        if response.status_code == 200:
            result = response.json()
            if result["success"]:
                print("✓ Function call interface: OK")
                print(f"  Response: {result['message']}")
                return True
            else:
                print(f"✗ Function call interface failed: {result.get('error', 'Unknown error')}")
                return False
        else:
            print(f"✗ Function call interface endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Function call interface error: {e}")
        return False

def main():
    """Run all tests"""
    print("Starting ROS 2 AI Agent Bridge tests...")
    print("=" * 50)

    # Check if server is running
    print("Checking if server is running...")
    try:
        response = requests.get(f"{BASE_URL}/health", timeout=5)
        if response.status_code == 200:
            print("✓ Server is running")
        else:
            print("✗ Server is not responding")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Server is not running. Please start the bridge server first.")
        print("Run: python rag/api/bridge_server.py")
        return False
    except Exception as e:
        print(f"✗ Error connecting to server: {e}")
        return False

    print()

    # Run all tests
    tests = [
        ("Health Check", test_health_endpoint),
        ("Available Services", test_available_services),
        ("Natural Language Translation", test_natural_language_translation),
        ("Direct Service Call", test_direct_service_call),
        ("Function Call Interface", test_function_call),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 50)
    print("TEST RESULTS SUMMARY:")

    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {test_name}: {status}")
        if not result:
            all_passed = False

    print()
    if all_passed:
        print("✓ All tests passed!")
        print("The ROS 2 AI Agent Bridge is working correctly.")
    else:
        print("✗ Some tests failed!")
        print("Please check the bridge server implementation.")

    return all_passed

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)