# Testing Workflow: Validating the Workspace Setup

This document outlines the process for testing that the workspace setup instructions work correctly in a fresh environment.

## Testing Objectives

The goal is to validate that a user with a properly installed ROS 2 environment can:
1. Create a new workspace following our instructions
2. Successfully run the "hello robot" publisher and subscriber
3. Verify all CLI tools work as expected
4. Complete the basic exercises without issues

## Testing Steps

### Step 1: Environment Verification
Before starting the workspace setup, verify the test environment has:
- ROS 2 Humble Hawksbill installed
- Python 3.10+ available
- Basic build tools (colcon, etc.)
- Network connectivity for any required packages

Commands to run:
```bash
ros2 --version
python3 --version
which colcon
```

### Step 2: Workspace Creation
Follow the exact steps from the workspace setup guide:
1. Create workspace directory: `mkdir -p ~/ros2_ws/src`
2. Navigate to workspace: `cd ~/ros2_ws`
3. Source ROS 2: `source /opt/ros/humble/setup.bash`
4. Build empty workspace: `colcon build`
5. Source workspace: `source install/setup.bash`

### Step 3: Example Node Setup
1. Copy the hello robot example files to the workspace:
   - `hello_robot_publisher.py`
   - `hello_robot_subscriber.py`
2. Place them in `~/ros2_ws/src/`
3. Build the workspace again: `colcon build`

### Step 4: Node Execution Test
1. Terminal 1 - Run publisher:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 src/hello_robot_publisher.py
   ```

2. Terminal 2 - Run subscriber:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 src/hello_robot_subscriber.py
   ```

3. Verify messages are passing from publisher to subscriber

### Step 5: CLI Tool Validation
Run the following commands to verify CLI tools work:
```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Get info about hello_robot topic
ros2 topic info /hello_robot

# Echo messages from the topic
ros2 topic echo /hello_robot std_msgs/msg/String
```

### Step 6: Validation Script Execution
Run the validation script to automatically test the setup:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/test_hello_robot.py
```

## Expected Results

- Publisher should output "Hello Robot! Message #X" every 0.5 seconds
- Subscriber should receive and display these messages
- CLI commands should return appropriate information
- Validation script should report success after receiving 5 messages
- No errors should occur during any of the operations

## Troubleshooting Common Issues

### Issue: "Command 'ros2' not found"
- Solution: Verify ROS 2 is installed and sourced

### Issue: Python import errors
- Solution: Check that rclpy is installed: `pip3 list | grep rclpy`

### Issue: Nodes can't communicate
- Solution: Verify both terminals have sourced the ROS 2 and workspace setup

### Issue: colcon build fails
- Solution: Check for syntax errors in Python files and proper package structure

## Success Criteria

This test is considered successful if:
- All setup steps complete without errors
- Publisher and subscriber communicate successfully
- CLI tools provide expected output
- Validation script passes
- User can follow along with the chapter exercises without issues

## Documentation of Results

When testing in an actual fresh environment, document:
- OS version and configuration
- ROS 2 installation method
- Any deviations from expected behavior
- Time required for each step
- User experience notes for improving documentation