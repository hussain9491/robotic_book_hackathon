# Chapter 1: Introduction to ROS 2 for Humanoid Robotics

## What is ROS 2 and Why is it the "Nervous System" of Humanoid Robots?

ROS 2 (Robot Operating System 2) serves as the communication backbone for humanoid robots, much like how the nervous system coordinates activities in biological organisms. It provides a middleware layer that enables different components of a humanoid robot to communicate effectively, regardless of the programming language, operating system, or physical location of those components.

For humanoid robots specifically, ROS 2 facilitates:
- Coordination between multiple joint controllers
- Sensor data integration from cameras, IMUs, and other sensors
- High-level planning and low-level control communication
- Simulation-to-reality transfer capabilities

## DDS (Data Distribution Service) and Real-time Communication Overview

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware. DDS provides:
- Real-time performance guarantees
- Quality of Service (QoS) policies for different communication needs
- Reliable message delivery
- Network transparency
- Language and platform independence

DDS enables ROS 2 to meet the demanding requirements of humanoid robotics where timing and reliability are critical for safety and performance.

## ROS 2 Workspace Setup

### Directory Structure
A typical ROS 2 workspace follows this structure:
```
workspace_folder/          # e.g., ~/ros2_ws
├── src/                   # Source code
├── build/                 # Build artifacts
├── install/               # Installation directory
└── log/                   # Log files
```

### Using colcon for Building
colcon is the build tool for ROS 2 workspaces. It can build packages in the src directory:
```bash
colcon build
source install/setup.bash
```

## ROS 2 CLI Basics

ROS 2 provides command-line tools for introspection and interaction:
- `ros2 node` - Node management and inspection
- `ros2 topic` - Topic inspection and interaction
- `ros2 service` - Service inspection and interaction
- `ros2 param` - Parameter management
- `ros2 action` - Action management

## Setting Up Your Development Environment

For this module, you'll need to set up your ROS 2 development environment. Follow the detailed steps in our [workspace setup guide](./workspace-setup-guide.md) to ensure your environment is properly configured.

## Learning Objectives

By the end of this chapter, students will be able to:
- Explain the role of ROS 2 in humanoid robot systems
- Set up a basic ROS 2 workspace
- Use basic ROS 2 command-line tools
- Run their first ROS 2 node

## Lab Exercise: "Hello Robot" Node

Let's create a simple publisher node that outputs "Hello Robot" messages to demonstrate basic ROS 2 functionality:

1. Navigate to your workspace src directory:
```bash
cd ~/ros2_ws/src
```

2. Create a new directory for our package:
```bash
mkdir hello_robot_pkg
cd hello_robot_pkg
```

3. Create the Python node file (we've already created this in our examples):
   - File: `hello_robot_publisher.py`
   - This node publishes "Hello Robot" messages to the `hello_robot` topic

4. To run the publisher:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/hello_robot_publisher.py
```

5. In another terminal, run the subscriber to see the messages:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/hello_robot_subscriber.py
```

You should see the publisher sending messages and the subscriber receiving them, demonstrating the basic publish-subscribe communication pattern in ROS 2.

## Key Concepts Summary

- ROS 2 provides the middleware for humanoid robot communication
- DDS ensures real-time and reliable communication
- Workspaces organize your ROS 2 development
- Nodes communicate through topics, services, and actions
- The publish-subscribe pattern is fundamental to ROS 2 communication