"""
Function definitions for OpenAI function calling to ROS 2 services
This file defines the available functions that AI agents can call to control the robot
"""

FUNCTION_DEFINITIONS = [
    {
        "name": "enable_joint_control",
        "description": "Enable joint control on the humanoid robot. Use this before sending movement commands.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "disable_joint_control",
        "description": "Disable joint control on the humanoid robot. Use this to stop all movements.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "move_joint_to_position",
        "description": "Move a specific joint to a target position. Joint positions are in radians.",
        "parameters": {
            "type": "object",
            "properties": {
                "joint_name": {
                    "type": "string",
                    "description": "Name of the joint to move (e.g., 'left_shoulder_pitch', 'right_elbow_joint', 'head_pan')"
                },
                "position": {
                    "type": "number",
                    "description": "Target position in radians",
                    "minimum": -3.14,
                    "maximum": 3.14
                }
            },
            "required": ["joint_name", "position"]
        }
    },
    {
        "name": "move_multiple_joints",
        "description": "Move multiple joints to target positions simultaneously. Joint positions are in radians.",
        "parameters": {
            "type": "object",
            "properties": {
                "joints": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "joint_name": {
                                "type": "string",
                                "description": "Name of the joint to move"
                            },
                            "position": {
                                "type": "number",
                                "description": "Target position in radians",
                                "minimum": -3.14,
                                "maximum": 3.14
                            }
                        },
                        "required": ["joint_name", "position"]
                    },
                    "description": "List of joints and their target positions"
                }
            },
            "required": ["joints"]
        }
    },
    {
        "name": "get_robot_state",
        "description": "Get the current state of the robot including joint positions and status.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "execute_preset_pose",
        "description": "Move the robot to a predefined pose such as 'ready', 'rest', 'attention', or 'wave'.",
        "parameters": {
            "type": "object",
            "properties": {
                "pose_name": {
                    "type": "string",
                    "enum": ["ready", "rest", "attention", "wave", "sit", "stand"],
                    "description": "Name of the predefined pose to execute"
                }
            },
            "required": ["pose_name"]
        }
    },
    {
        "name": "get_joint_limits",
        "description": "Get the position, velocity, and effort limits for a specific joint.",
        "parameters": {
            "type": "object",
            "properties": {
                "joint_name": {
                    "type": "string",
                    "description": "Name of the joint to get limits for"
                }
            },
            "required": ["joint_name"]
        }
    },
    {
        "name": "check_safety_violations",
        "description": "Check if any safety constraints are being violated.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "emergency_stop",
        "description": "Immediately stop all robot movements and disable joint control for safety.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    }
]

# Predefined poses for the humanoid robot
PREDEFINED_POSES = {
    "ready": {
        "left_shoulder_pitch": 0.2,
        "left_shoulder_roll": 0.1,
        "left_elbow_joint": -0.5,
        "right_shoulder_pitch": 0.2,
        "right_shoulder_roll": -0.1,
        "right_elbow_joint": -0.5,
        "left_hip_pitch": 0.0,
        "right_hip_pitch": 0.0,
        "left_knee_joint": 0.0,
        "right_knee_joint": 0.0
    },
    "rest": {
        "left_shoulder_pitch": 0.0,
        "left_shoulder_roll": 0.0,
        "left_elbow_joint": 0.0,
        "right_shoulder_pitch": 0.0,
        "right_shoulder_roll": 0.0,
        "right_elbow_joint": 0.0,
        "left_hip_pitch": 0.0,
        "right_hip_pitch": 0.0,
        "left_knee_joint": 0.0,
        "right_knee_joint": 0.0
    },
    "attention": {
        "left_shoulder_pitch": 0.0,
        "left_shoulder_roll": 0.0,
        "left_elbow_joint": -1.57,
        "right_shoulder_pitch": 0.0,
        "right_shoulder_roll": 0.0,
        "right_elbow_joint": -1.57,
        "left_hip_pitch": 0.0,
        "right_hip_pitch": 0.0,
        "left_knee_joint": 0.0,
        "right_knee_joint": 0.0
    },
    "wave": {
        "left_shoulder_pitch": 0.2,
        "left_shoulder_roll": 0.1,
        "left_elbow_joint": -0.5,
        "right_shoulder_pitch": 1.0,
        "right_shoulder_roll": 0.0,
        "right_elbow_joint": -1.5,
        "left_hip_pitch": 0.0,
        "right_hip_pitch": 0.0,
        "left_knee_joint": 0.0,
        "right_knee_joint": 0.0
    }
}