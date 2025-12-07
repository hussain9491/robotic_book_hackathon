# Chapter 5: Bridging AI Agents to ROS 2

This chapter explains how to connect AI agents to ROS 2 to bridge high-level planning with low-level robot control. We'll build a FastAPI server that serves as an interface between AI agents (like OpenAI's GPT models) and ROS 2 systems.

## Overview

The integration between AI agents and ROS 2 systems requires a bridge that can:
- Translate natural language commands into ROS 2 service calls
- Provide function calling interfaces for precise control
- Validate commands for safety before execution
- Handle errors gracefully and provide feedback

## The Bridge Architecture

The bridge consists of a FastAPI server that:
1. Receives requests from AI agents
2. Translates them to appropriate ROS 2 service calls
3. Returns results back to the AI agent
4. Implements safety validation layers

```
AI Agent (OpenAI)
    ↓ (natural language or function calls)
FastAPI Bridge Server
    ↓ (ROS 2 service calls)
ROS 2 Nodes
    ↓ (robot control)
Physical Robot
```

## Implementation Components

### 1. FastAPI Server (`rag/api/bridge_server.py`)

The server provides several key endpoints:

- `/translate-command`: Translates natural language to ROS 2 services
- `/call-service`: Direct interface to ROS 2 services
- `/function-call`: OpenAI function calling interface
- `/available-services`: Lists available services

### 2. Service Mapping

The bridge maps AI agent requests to ROS 2 services:

```python
function_map = {
    "enable_joint_control": {
        "service_name": "/joint_control/enable",
        "service_type": "std_srvs/SetBool",
        "request_data": {"data": True}
    },
    "move_joint_to_position": {
        "service_name": "/joint_control/move_to_position",
        "service_type": "sensor_msgs/JointState",
        "request_data": arguments
    }
}
```

### 3. Safety Validation

Before executing commands, the system validates:
- Position limits
- Velocity limits
- Joint constraints
- Robot state

## OpenAI Function Calling Integration

The bridge supports OpenAI's function calling API, allowing the AI agent to call specific robot functions:

```python
functions = [
    {
        "name": "enable_joint_control",
        "description": "Enable joint control on the robot",
        "parameters": {
            "type": "object",
            "properties": {},
        }
    },
    {
        "name": "move_joint_to_position",
        "description": "Move a joint to a specific position",
        "parameters": {
            "type": "object",
            "properties": {
                "joint_name": {"type": "string"},
                "position": {"type": "number"}
            },
            "required": ["joint_name", "position"]
        }
    }
]
```

## Safety Considerations

### Position and Velocity Limits
- All joint positions are validated against physical limits
- Velocity commands are checked for safety

### State Validation
- Robot state is checked before executing commands
- Emergency stop functionality is available

### Error Handling
- Graceful error handling for invalid commands
- Detailed error messages for debugging

## Testing the Integration

The system includes comprehensive testing capabilities:

1. Health checks for server status
2. Service availability verification
3. Natural language command processing
4. Direct service call validation
5. Function calling interface testing

## Real-World Deployment

In a real deployment, the bridge server would connect to actual ROS 2 nodes controlling the physical robot. The mock implementations provided are for testing and demonstration purposes.

## Chapter Summary

This chapter covered:
- The architecture of the AI agent to ROS 2 bridge
- Implementation of the FastAPI server
- Safety validation mechanisms
- OpenAI function calling integration
- Testing procedures

The bridge enables AI agents to control ROS 2-based robots safely and effectively, bridging the gap between high-level planning and low-level execution.