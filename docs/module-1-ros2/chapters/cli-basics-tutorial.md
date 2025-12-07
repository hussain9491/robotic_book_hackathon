# ROS 2 CLI Basics Tutorial

This tutorial covers the fundamental ROS 2 command-line interface tools that are essential for working with humanoid robot systems.

## Overview of ROS 2 CLI Tools

ROS 2 provides several command-line tools for inspecting and interacting with your robot system:

- `ros2 node` - Manage and inspect ROS 2 nodes
- `ros2 topic` - Work with topics and messages
- `ros2 service` - Work with services
- `ros2 param` - Manage node parameters
- `ros2 action` - Work with actions
- `ros2 pkg` - Package management
- `ros2 run` - Run nodes directly

## Node Management with `ros2 node`

### Listing Active Nodes
To see all currently running nodes:
```bash
ros2 node list
```

### Getting Information About a Specific Node
To get detailed information about a node:
```bash
ros2 node info <node_name>
```

Example:
```bash
ros2 node info hello_robot_publisher
```

This will show:
- Subscribed topics
- Published topics
- Service servers
- Service clients
- Action servers
- Action clients

## Topic Management with `ros2 topic`

### Listing Active Topics
To see all currently active topics:
```bash
ros2 topic list
```

### Getting Topic Information
To get information about a specific topic:
```bash
ros2 topic info <topic_name>
```

Example:
```bash
ros2 topic info /hello_robot
```

### Echoing Topic Messages
To see messages being published to a topic in real-time:
```bash
ros2 topic echo <topic_name> <message_type>
```

Example:
```bash
ros2 topic echo /hello_robot std_msgs/msg/String
```

### Publishing Messages to a Topic
To publish a message to a topic directly from the command line:
```bash
ros2 topic pub <topic_name> <message_type> <message_data>
```

Example:
```bash
ros2 topic pub /hello_robot std_msgs/msg/String "data: 'Hello from CLI'"
```

### Finding Topic Types
To determine the message type of a topic:
```bash
ros2 topic type <topic_name>
```

## Service Management with `ros2 service`

### Listing Active Services
To see all currently available services:
```bash
ros2 service list
```

### Getting Service Information
To get information about a specific service:
```bash
ros2 service info <service_name>
```

### Calling a Service
To call a service directly from the command line:
```bash
ros2 service call <service_name> <service_type> <request_data>
```

Example:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### Finding Service Types
To determine the service type:
```bash
ros2 service type <service_name>
```

## Parameter Management with `ros2 param`

### Listing Parameters for a Node
To see all parameters for a specific node:
```bash
ros2 param list <node_name>
```

### Getting Parameter Values
To get the value of a specific parameter:
```bash
ros2 param get <node_name> <param_name>
```

### Setting Parameter Values
To set a parameter value:
```bash
ros2 param set <node_name> <param_name> <value>
```

Example:
```bash
ros2 param set hello_robot_publisher use_sim_time true
```

## Action Management with `ros2 action`

### Listing Active Actions
To see all currently available actions:
```bash
ros2 action list
```

### Getting Action Information
To get information about a specific action:
```bash
ros2 action info <action_name>
```

### Sending Action Goals
To send a goal to an action server:
```bash
ros2 action send_goal <action_name> <action_type> <goal_data>
```

Example:
```bash
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

## Package Management with `ros2 pkg`

### Listing Available Packages
To see all available packages:
```bash
ros2 pkg list
```

### Getting Package Information
To get information about a specific package:
```bash
ros2 pkg info <package_name>
```

## Running Nodes with `ros2 run`

To run a node directly without building a launch file:
```bash
ros2 run <package_name> <executable_name>
```

Example (for Python nodes, you would typically run the Python file directly, but for compiled nodes):
```bash
ros2 run demo_nodes_cpp talker
```

## Practical Exercise: Exploring Your Hello Robot System

Now that you have your hello robot publisher and subscriber running, let's explore the system using CLI tools:

1. **List active nodes**:
   ```bash
   ros2 node list
   ```
   You should see `hello_robot_publisher` and `hello_robot_subscriber`.

2. **Get information about the publisher**:
   ```bash
   ros2 node info hello_robot_publisher
   ```
   You should see it's publishing to `/hello_robot`.

3. **List active topics**:
   ```bash
   ros2 topic list
   ```
   You should see `/hello_robot` among other system topics.

4. **Get topic information**:
   ```bash
   ros2 topic info /hello_robot
   ```
   This shows the topic's type and who's publishing/subscribing.

5. **Echo messages from the topic**:
   Make sure your publisher is running, then in another terminal:
   ```bash
   ros2 topic echo /hello_robot std_msgs/msg/String
   ```
   You should see the same messages that your subscriber is receiving.

## Tips for Humanoid Robot Development

- Use `ros2 node list` to check if all required robot nodes are running
- Use `ros2 topic list` to discover what data is available from your robot
- Use `ros2 topic echo` to monitor sensor data without writing code
- Use `ros2 param` to tune controller parameters at runtime
- Use `ros2 service call` to trigger robot behaviors for testing

## Troubleshooting CLI Commands

If CLI commands return "command not found":
- Make sure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
- Check that your ROS_DISTRO environment variable is set: `echo $ROS_DISTRO`

If commands don't find your nodes/topics:
- Ensure nodes are actually running
- Check that you're in the right namespace if using one
- Verify that the correct ROS_DOMAIN_ID is set if using multiple systems