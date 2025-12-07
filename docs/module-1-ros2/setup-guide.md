# ROS 2 Humble Hawksbill Development Environment Setup

This document outlines the steps required to set up a ROS 2 Humble Hawksbill development environment for the Physical AI & Humanoid Robotics book.

## Prerequisites

- Ubuntu 22.04 (recommended) or Windows 10/11 with WSL2
- At least 4GB RAM
- 20GB free disk space
- Python 3.10+

## Installation Steps

### 1. Install ROS 2 Humble Hawksbill

#### On Ubuntu 22.04:

```bash
# Set locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo apt install ros-humble-rclpy
sudo apt install ros-humble-urdf
sudo apt install ros-humble-xacro
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-joint-state-publisher-gui
```

### 2. Environment Setup

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to bashrc to source automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Install Required Tools

```bash
# colcon for building ROS 2 workspaces
sudo apt install python3-colcon-common-extensions

# rclpy for Python ROS 2 client library
sudo apt install ros-humble-rclpy

# RViz2 for visualization
sudo apt install ros-humble-rviz2

# URDF tools for robot description validation
sudo apt install ros-humble-urdf
sudo apt install ros-humble-check-urdf

# Other useful tools
sudo apt install ros-humble-ros-base
```

### 4. Create a ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Verification

To verify the installation, run:

```bash
# Check ROS 2 version
ros2 --version

# Test basic functionality
ros2 run demo_nodes_cpp talker
```

## Required Tools Summary

- **colcon**: Build system for ROS 2 workspaces
- **rclpy**: Python client library for ROS 2
- **RViz2**: 3D visualization tool for ROS 2
- **check_urdf**: URDF validation tool
- **ros2**: Main ROS 2 command-line interface

## Troubleshooting

- If packages cannot be found, ensure the ROS 2 apt repository is properly added
- If environment variables are not set, make sure to source the setup.bash file
- For Python-related issues, ensure Python 3.10+ is installed and properly configured