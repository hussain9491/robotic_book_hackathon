# Chapter 4: URDF: Modeling a Humanoid Robot

## Understanding URDF for Bipedal Robots

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. For humanoid robots, URDF defines:
- Physical structure (links)
- Joints connecting the links
- Inertial properties
- Visual and collision representations

URDF is fundamental to robotics simulation, visualization, and control. It provides a complete description of a robot's physical properties that can be used by various ROS tools.

## Links, Joints, and Inertial Properties

### Links
Links represent rigid bodies in the robot. For a humanoid, these might include:
- Torso (body)
- Head
- Upper and lower arms
- Upper and lower legs
- Hands and feet

Each link must define:
- Visual properties (how it looks)
- Collision properties (for physics simulation)
- Inertial properties (mass, center of mass, moments of inertia)

Example link definition:
```xml
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="red"/>
    <origin rpy="0 0 0" xyz="0 0 -0.15"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin rpy="0 0 0" xyz="0 0 -0.15"/>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

### Joints
Joints define how links connect and move relative to each other:
- Fixed joints (no movement)
- Revolute joints (rotation around single axis)
- Continuous joints (unlimited rotation)
- Prismatic joints (linear movement)

Joint types for humanoid robots:
- Revolute joints for most articulation points (shoulders, elbows, hips, knees)
- Fixed joints for permanent connections (head to neck)

Example joint definition:
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0.0 0.0 -0.3"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.0" upper="0.0" effort="100" velocity="3.0"/>
</joint>
```

### Inertial Properties
These define the mass, center of mass, and inertia tensor for physics simulation. Proper inertial properties are critical for realistic simulation and control.

## Visual vs Collision Meshes

### Visual Elements
Visual meshes define how the robot appears in visualization tools like RViz. These can be:
- Simple geometric shapes (boxes, cylinders, spheres) for basic visualization
- Complex meshes for detailed appearance
- Color and material properties

### Collision Elements
Collision meshes define the simplified geometry used for collision detection. These should be:
- Simpler than visual meshes for performance
- Conservative (larger than actual robot) for safety
- Defined for all links that might contact the environment

## Building a Minimal Humanoid URDF Model

Let's create a minimal humanoid model with torso, 2 arms, and 2 legs:

```xml
<?xml version="1.0"?>
<robot name="minimal_humanoid">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0.0 0.0 0.35"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
      <origin rpy="0 0 0" xyz="0 0 -0.15"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 -0.15"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.0 0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red"/>
      <origin rpy="0 0 0" xyz="0 0 -0.15"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 -0.15"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-2.0" upper="0.0" effort="100" velocity="3.0"/>
  </joint>

  <!-- Similar definitions for right arm, legs, etc. -->
</robot>
```

## Best Practices for Humanoid URDF Models

### Link Naming Conventions
- Use descriptive names (e.g., `left_upper_arm`, `right_hip_yaw`)
- Follow consistent patterns across the robot
- Use underscores to separate components

### Joint Limitations
- Set realistic joint limits based on physical constraints
- Consider safety margins in limit settings
- Document the reasoning behind limit choices

### Mass and Inertia
- Use realistic mass values based on actual components
- Ensure inertial tensors are physically valid
- Consider the impact of payloads on mass properties

## Validating URDF Models

### Using check_urdf
The `check_urdf` command validates URDF syntax and structure:
```bash
check_urdf /path/to/robot.urdf
```

This command will:
- Verify XML syntax
- Check for proper link/joint connections
- Validate joint limits and types
- Report any errors or warnings

### Visualization in RViz
Load your URDF in RViz to visually inspect the model:
```bash
ros2 run rviz2 rviz2
```
In RViz:
- Add a RobotModel display
- Set the description topic to `/robot_description`
- Verify all links appear correctly positioned

## Integrating with ROS 2 Control Interfaces

URDF models work with ROS 2 control packages to enable actual robot control. Key integration points include:

### ros2_control
The `ros2_control` framework uses URDF to understand robot structure and connect to hardware. Transmission elements in URDF define how joints connect to controllers.

### Joint State Publisher
The `joint_state_publisher` and `robot_state_publisher` nodes use URDF to publish transforms for visualization and navigation.

## Learning Objectives

By the end of this chapter, students will be able to:
- Create URDF models for humanoid robots with proper link and joint definitions
- Define visual, collision, and inertial properties for robot components
- Validate URDF models using ROS 2 tools
- Visualize robots in RViz with proper configuration
- Apply best practices for humanoid robot modeling

## Lab Exercise: Complete Humanoid Robot Model

In this lab, you will:
1. Create a complete minimal humanoid URDF model with torso, arms, and legs
2. Validate the model using `check_urdf`
3. Visualize the model in RViz using the provided configuration
4. Add proper materials and colors to distinguish different body parts
5. Set realistic joint limits based on human anatomy

Use the example URDF provided in this chapter as a starting point, and extend it to include all major body parts of a humanoid robot. Pay special attention to proper joint types and limits that reflect the mechanical constraints of a real humanoid robot.