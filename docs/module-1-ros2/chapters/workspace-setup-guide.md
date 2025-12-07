# Step-by-Step ROS 2 Workspace Setup Guide

This guide provides detailed instructions for setting up a ROS 2 workspace for humanoid robotics development, with screenshots and verification steps.

## Prerequisites

Before setting up your ROS 2 workspace, ensure you have:
- ROS 2 Humble Hawksbill installed (see setup guide)
- Python 3.10 or higher
- Basic command-line familiarity
- At least 5GB free disk space

## Creating Your First Workspace

### 1. Create Workspace Directory

Open a terminal and create a new directory for your ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

This creates the standard workspace structure:
- `~/ros2_ws/` - Main workspace directory
- `~/ros2_ws/src/` - Source code directory (where you'll place packages)
- `~/ros2_ws/build/` - Build artifacts (created during compilation)
- `~/ros2_ws/install/` - Installation directory (created after building)
- `~/ros2_ws/log/` - Log files (created during building/running)

### 2. Source ROS 2 Environment

Before working with ROS 2, you need to source the ROS 2 installation:

```bash
source /opt/ros/humble/setup.bash
```

For convenience, add this to your bashrc to source automatically:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Build the Workspace

Even though your src directory is empty, build the workspace to initialize it:

```bash
colcon build
```

This command will:
- Create build/ and install/ directories
- Generate setup files for sourcing the workspace
- Verify your ROS 2 installation is working

### 4. Source the Workspace

After building, source the workspace to use any packages you create:

```bash
source install/setup.bash
```

## Verification Steps

To verify your workspace is set up correctly:

1. Check ROS 2 environment variables:
```bash
printenv | grep ROS
```

2. Verify workspace is in the path:
```bash
echo $AMENT_PREFIX_PATH
```

3. Test basic ROS 2 functionality:
```bash
ros2 run demo_nodes_cpp talker
```

In another terminal (remember to source both ROS 2 and your workspace):
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages being published from talker to listener.

## Troubleshooting Common Issues

### Issue: Command 'ros2' not found
- Solution: Make sure you sourced the ROS 2 setup: `source /opt/ros/humble/setup.bash`

### Issue: colcon command not found
- Solution: Install colcon: `sudo apt install python3-colcon-common-extensions`

### Issue: Permission denied when building
- Solution: Make sure you're not running as root, and check file permissions

### Issue: Workspace packages not found
- Solution: Always remember to source your workspace after building: `source install/setup.bash`

## Best Practices

- Always source ROS 2 and your workspace before developing
- Use a new terminal or re-source when switching between projects
- Regularly run `colcon build` after making changes to packages
- Use `--packages-select <package_name>` to build specific packages
- Clean build artifacts with `rm -rf build/ install/ log/` if needed

## Next Steps

With your workspace set up, you're ready to create your first ROS 2 package and node. Continue to the next chapter to learn how to create and run a "Hello Robot" node.