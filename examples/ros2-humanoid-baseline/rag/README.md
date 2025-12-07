# ROS 2 AI Agent Bridge

This FastAPI server provides an interface between AI agents (like OpenAI) and ROS 2 systems. It translates natural language commands into ROS 2 service calls and provides a function calling interface for precise robot control.

## Overview

The bridge server enables AI agents to control a ROS 2-based humanoid robot through:

1. Natural language command translation
2. Direct ROS 2 service calls
3. OpenAI function calling interface

## Architecture

```
AI Agent (OpenAI)
    ↓ (natural language)
Bridge Server (FastAPI)
    ↓ (ROS 2 service calls)
ROS 2 Nodes (Robot Control)
```

## Endpoints

### `/translate-command` (POST)
- Translates natural language commands to ROS 2 service calls
- Example: "Enable joint control" → calls `/joint_control/enable`

### `/call-service` (POST)
- Direct interface to ROS 2 services
- Example: Call any ROS 2 service with specific parameters

### `/function-call` (POST)
- OpenAI function calling interface
- Maps function names to ROS 2 services

### `/available-services` (GET)
- Lists available ROS 2 services

## Usage

### 1. Start the Bridge Server

```bash
pip install -r requirements.txt
python rag/api/bridge_server.py
```

### 2. Example API Calls

Translate a natural language command:
```bash
curl -X POST http://localhost:8000/translate-command \
  -H "Content-Type: application/json" \
  -d '{
    "command": "Enable joint control",
    "parameters": {},
    "robot_id": "default_robot"
  }'
```

Call a service directly:
```bash
curl -X POST http://localhost:8000/call-service \
  -H "Content-Type: application/json" \
  -d '{
    "service_name": "/joint_control/enable",
    "service_type": "std_srvs/SetBool",
    "request_data": {"data": true}
  }'
```

### 3. OpenAI Function Calling

The server supports OpenAI's function calling API. Define functions like:

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

## Safety Features

- Command validation before execution
- Position and velocity limits enforcement
- Error handling and logging
- Robot state monitoring

## Integration with AI Agents

The bridge server is designed to work with OpenAI's Assistants API or Chat Completions API with function calling. The AI agent can call functions like:

- `enable_joint_control()` - Enable robot joint control
- `disable_joint_control()` - Disable robot joint control
- `move_joint_to_position(joint_name, position)` - Move specific joint
- `get_robot_state()` - Get current robot state

## Real Implementation Notes

In a real ROS 2 environment, you would replace the `MockROS2ServiceClient` with actual ROS 2 service clients that communicate with your robot's control nodes. The mock implementation is provided for testing and demonstration purposes.

## Testing

To test the bridge server without a real ROS 2 environment:

1. Install dependencies: `pip install -r requirements.txt`
2. Start the server: `python rag/api/bridge_server.py`
3. Make API calls as shown in the examples above