# Testing URDF Visualization in RViz2

This document explains how to test and validate the humanoid robot URDF model in RViz2.

## Prerequisites

Before testing the URDF visualization, ensure you have:
- ROS 2 Humble Hawksbill installed
- RViz2 installed (`ros-humble-rviz2`)
- robot_state_publisher installed (`ros-humble-robot-state-publisher`)
- joint_state_publisher installed (`ros-humble-joint-state-publisher`)

## Manual Testing Process

### 1. Start the Robot State Publisher

Open a terminal and run:

```bash
cd ~/ros2_ws  # or your workspace directory
source install/setup.bash

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat src/minimal_humanoid.urdf)
```

### 2. Start the Joint State Publisher (GUI)

In another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### 3. Start RViz2

In another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash

rviz2 -d src/minimal_humanoid.rviz
```

### 4. Verification Steps

In RViz2, verify:
- The robot model appears in the 3D view
- All links are visible and properly connected
- Joint movements are reflected when using the joint state publisher GUI
- No TF transform errors in the terminal running robot_state_publisher
- Proper colors and materials are applied

## Using the Launch File

Alternatively, you can use the provided launch file to start all components at once:

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch src/launch/view_humanoid.launch.py
```

This will start:
- robot_state_publisher with the URDF loaded
- joint_state_publisher_gui for interactive joint control
- RViz2 with the pre-configured visualization

## Expected Results

When the visualization is working correctly:
- The humanoid robot model should appear in RViz2
- You should see the robot's links (torso, arms, legs) in the correct positions
- The robot should be positioned relative to the `base_link` frame
- Joint controllers in the GUI should correspond to the robot's actual joints
- TF transforms should be published without errors

## Troubleshooting Common Issues

### Issue: "No transform from [link_name] to [frame]"
- Solution: Verify all joints are properly defined with correct parent/child relationships

### Issue: Robot model not appearing in RViz
- Solution: Check that the robot_description parameter is set correctly
- Check that the robot_state_publisher node is running without errors

### Issue: Wrong colors or materials
- Solution: Verify material definitions are included in the URDF

### Issue: Joint limits not respected
- Solution: Check that joint limits are properly defined in the URDF

## Validation Checklist

Complete this checklist to verify the URDF visualization:

- [ ] Robot model loads without errors
- [ ] All links are visible in RViz
- [ ] Joint relationships are correct
- [ ] TF transforms are published correctly
- [ ] Joint movement controls work
- [ ] Materials and colors display properly
- [ ] No warnings or errors in the console output

## Success Criteria

The URDF visualization test is considered successful when:
1. The robot model appears correctly in RViz
2. All joints can be manipulated using the joint state publisher GUI
3. TF transforms are published without errors
4. The robot's kinematic structure is visually correct
5. All visual elements (colors, shapes) are displayed properly