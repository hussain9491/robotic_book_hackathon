# Chapter 2: ROS 2 Nodes, Topics, and Services

## ROS 2 Communication Ecosystem

ROS 2 provides several communication patterns for different use cases:
- **Topics**: Publish/subscribe pattern for asynchronous communication
- **Services**: Request/response pattern for synchronous communication
- **Actions**: Goal-based communication with feedback and status

These communication patterns form the foundation of distributed robot control systems, enabling different components to coordinate effectively.

## Building Python Nodes Using rclpy

rclpy is the Python client library for ROS 2. It allows you to create ROS 2 nodes in Python by providing:
- Node creation and lifecycle management
- Publisher and subscriber interfaces
- Service server and client interfaces
- Parameter handling
- Logging capabilities

### Basic Node Structure

Every ROS 2 node follows a standard structure:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Publishers & Subscribers

Publishers and subscribers enable asynchronous communication through topics. Publishers send messages to topics, while subscribers receive messages from topics. Quality of Service (QoS) profiles allow fine-tuning of communication behavior.

### Publisher Example

Here's a publisher that sends joint state messages for a humanoid robot:

```python
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
```

### Subscriber Example

Here's a subscriber that receives and logs joint state messages:

```python
class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

    def joint_state_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.name)} joint states')
        # Process joint state data
```

## Using QoS Profiles

Quality of Service (QoS) profiles in ROS 2 define the communication behavior between publishers and subscribers. Key QoS settings include:
- Reliability: Best effort vs. reliable delivery
- Durability: Volatile vs. transient local
- History: Keep last N messages vs. keep all messages
- Deadline: Maximum time between consecutive messages
- Lifespan: Maximum time a message can exist

### QoS Profile Examples for Humanoid Robots

For different types of communication in humanoid robots, you'll use different QoS profiles:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For control commands requiring reliability
control_qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# For sensor data where some loss is acceptable
sensor_qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)
```

## Writing Service Servers & Clients

Services provide synchronous request/response communication. Service servers process requests and return responses, while service clients send requests and wait for responses.

### Service Server Example

```python
from example_interfaces.srv import SetBool

class JointControlServer(Node):
    def __init__(self):
        super().__init__('joint_control_server')
        self.srv = self.create_service(
            SetBool,  # In practice, use custom service type
            'command_joint_position',
            self.joint_command_callback
        )

    def joint_command_callback(self, request, response):
        # Process the joint command
        response.success = True
        response.message = "Command processed"
        return response
```

### Service Client Example

```python
class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')
        self.client = self.create_client(SetBool, 'command_joint_position')

    def send_command(self, command_data):
        request = SetBool.Request()
        request.data = command_data

        future = self.client.call_async(request)
        # Handle response asynchronously
```

## Understanding Message Flow in Humanoid Control Loops

In humanoid robot control, different components communicate through ROS 2 topics and services to coordinate movement, sensor data, and high-level planning. A typical control loop might look like:

1. Sensor nodes publish sensor data (IMU, joint encoders, cameras)
2. Perception nodes process sensor data and publish processed information
3. Planning nodes make decisions based on processed data
4. Control nodes send commands to actuators
5. Actuator nodes execute commands and provide feedback

## Learning Objectives

By the end of this chapter, students will be able to:
- Create ROS 2 nodes using rclpy
- Implement publishers and subscribers for asynchronous communication
- Configure QoS profiles for different communication needs
- Build service servers and clients for synchronous communication
- Design message flows for humanoid robot control systems

## Lab Exercise: Joint State Publisher and Controller

In this lab, you will create a complete publisher-subscriber-service system:

1. Create a publisher that sends joint state messages for a humanoid robot
2. Create a subscriber that receives and logs these messages
3. Create a service server that accepts joint position commands
4. Create a service client that sends commands to the server
5. Test the system to ensure reliable communication

Use the example code provided in this chapter as a starting point, and modify it to work with the joint names and ranges appropriate for a humanoid robot. Implement proper error handling and logging to make your nodes robust.

## Key Takeaways

- Topics provide asynchronous, decoupled communication
- Services provide synchronous request/response communication
- QoS profiles allow fine-tuning of communication behavior
- Nodes should be designed to handle communication failures gracefully
- For humanoid robots, choose appropriate QoS settings based on the criticality of the data