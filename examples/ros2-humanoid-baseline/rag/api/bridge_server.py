#!/usr/bin/env python3
"""
FastAPI server to interface between OpenAI API and ROS 2
This server receives natural language commands from an AI agent and converts them to ROS 2 service calls.
"""

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import asyncio
import logging
import os
import sys
import importlib.util

# Import function definitions and safety validator
spec = importlib.util.spec_from_file_location("function_definitions",
    os.path.join(os.path.dirname(__file__), "function_definitions.py"))
function_definitions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(function_definitions)
FUNCTION_DEFINITIONS = function_definitions.FUNCTION_DEFINITIONS
PREDEFINED_POSES = function_definitions.PREDEFINED_POSES

spec = importlib.util.spec_from_file_location("safety_validator",
    os.path.join(os.path.dirname(__file__), "safety_validator.py"))
safety_validator_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(safety_validator_module)
SafetyValidator = safety_validator_module.SafetyValidator

spec = importlib.util.spec_from_file_location("nlp_mapper",
    os.path.join(os.path.dirname(__file__), "nlp_mapper.py"))
nlp_mapper_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nlp_mapper_module)
NLPCommandMapper = nlp_mapper_module.NLPCommandMapper

spec = importlib.util.spec_from_file_location("logging_utils",
    os.path.join(os.path.dirname(__file__), "logging_utils.py"))
logging_utils_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logging_utils_module)
SafetyViolationLogger = logging_utils_module.SafetyViolationLogger

# Initialize FastAPI app
app = FastAPI(
    title="ROS 2 AI Agent Bridge",
    description="API to translate natural language commands to ROS 2 service calls",
    version="1.0.0"
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Define request/response models
class NaturalLanguageCommand(BaseModel):
    """Request model for natural language commands"""
    command: str
    parameters: Optional[Dict[str, Any]] = {}
    robot_id: Optional[str] = "default_robot"

class ServiceCallRequest(BaseModel):
    """Request model for direct service calls"""
    service_name: str
    service_type: str
    request_data: Dict[str, Any]

class ServiceCallResponse(BaseModel):
    """Response model for service calls"""
    success: bool
    message: str
    response_data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

# Mock ROS 2 service client (in a real implementation, this would interface with actual ROS 2 nodes)
class MockROS2ServiceClient:
    """Mock class to simulate ROS 2 service calls"""

    def __init__(self):
        self.services = {
            "/joint_control/enable": self._handle_joint_control_enable,
            "/joint_control/disable": self._handle_joint_control_disable,
            "/joint_control/move_to_position": self._handle_move_to_position,
            "/robot_state/get": self._handle_get_robot_state,
            "/joint_limits/get": self._handle_get_joint_limits,
            "/safety/check": self._handle_check_safety,
            "/emergency_stop": self._handle_emergency_stop,
        }
        self.joint_states = {
            "head_pan": 0.0,
            "left_shoulder_pitch": 0.2,
            "left_shoulder_roll": 0.1,
            "left_elbow_joint": -0.5,
            "right_shoulder_pitch": 0.2,
            "right_shoulder_roll": -0.1,
            "right_elbow_joint": -0.5,
            "left_hip_pitch": 0.0,
            "left_knee_joint": 0.0,
            "right_hip_pitch": 0.0,
            "right_knee_joint": 0.0
        }
        self.joint_limits = {
            "head_pan": {"min": -1.57, "max": 1.57, "velocity": 2.0, "effort": 100.0},
            "left_shoulder_pitch": {"min": -2.0, "max": 2.0, "velocity": 2.0, "effort": 100.0},
            "left_shoulder_roll": {"min": -1.57, "max": 1.57, "velocity": 2.0, "effort": 100.0},
            "left_elbow_joint": {"min": -2.0, "max": 0.0, "velocity": 2.0, "effort": 100.0},
            "right_shoulder_pitch": {"min": -2.0, "max": 2.0, "velocity": 2.0, "effort": 100.0},
            "right_shoulder_roll": {"min": -1.57, "max": 1.57, "velocity": 2.0, "effort": 100.0},
            "right_elbow_joint": {"min": -2.0, "max": 0.0, "velocity": 2.0, "effort": 100.0},
            "left_hip_pitch": {"min": -1.0, "max": 1.0, "velocity": 1.5, "effort": 200.0},
            "left_knee_joint": {"min": 0.0, "max": 2.0, "velocity": 1.5, "effort": 200.0},
            "right_hip_pitch": {"min": -1.0, "max": 1.0, "velocity": 1.5, "effort": 200.0},
            "right_knee_joint": {"min": 0.0, "max": 2.0, "velocity": 1.5, "effort": 200.0}
        }
        self.enabled = False
        # Initialize safety validator
        self.safety_validator = SafetyValidator()
        # Update safety validator with current states
        self.safety_validator.update_current_state(self.joint_states)

    async def call_service(self, service_name: str, service_type: str, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Call a ROS 2 service with the given data"""
        logger.info(f"Calling service: {service_name} with type: {service_type} and data: {request_data}")

        if service_name in self.services:
            try:
                # Call the appropriate handler
                result = await self.services[service_name](request_data)
                return {
                    "success": True,
                    "response_data": result,
                    "message": f"Successfully called {service_name}"
                }
            except Exception as e:
                logger.error(f"Error calling service {service_name}: {str(e)}")
                return {
                    "success": False,
                    "error": str(e),
                    "message": f"Error calling service {service_name}"
                }
        else:
            error_msg = f"Service {service_name} not found"
            logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg,
                "message": error_msg
            }

    async def _handle_joint_control_enable(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle joint control enable service"""
        self.enabled = True
        return {"enabled": True, "message": "Joint control enabled"}

    async def _handle_joint_control_disable(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle joint control disable service"""
        self.enabled = False
        return {"enabled": False, "message": "Joint control disabled"}

    async def _handle_move_to_position(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle move to position service"""
        if not self.enabled:
            return {"success": False, "message": "Joint control is not enabled. Enable control first."}

        # Handle both single joint and multiple joints
        joints = request_data.get("joints", {})
        if not joints and "joint_name" in request_data and "position" in request_data:
            # Single joint movement
            joint_name = request_data["joint_name"]
            position = request_data["position"]

            # Use safety validator to check if command is safe
            is_safe, message = self.safety_validator.validate_single_joint_command(joint_name, position)
            if not is_safe:
                return {"success": False, "message": f"Safety validation failed: {message}"}

            # Update joint state
            self.joint_states[joint_name] = position
            # Update safety validator with new state
            self.safety_validator.update_current_state(self.joint_states)
            return {
                "success": True,
                "joints_moved": [joint_name],
                "message": f"Moved {joint_name} to position {position}"
            }
        elif isinstance(joints, list):
            # Multiple joint movement
            # Use safety validator to check if all commands are safe
            is_safe, message, violations = self.safety_validator.validate_multiple_joint_commands(joints)
            if not is_safe:
                return {
                    "success": False,
                    "message": f"Safety validation failed: {message}",
                    "violations": violations
                }

            # Execute the movements
            moved_joints = []
            for joint_data in joints:
                joint_name = joint_data.get("joint_name")
                position = joint_data.get("position")
                if joint_name and position is not None:
                    self.joint_states[joint_name] = position
                    moved_joints.append(joint_name)

            # Update safety validator with new states
            self.safety_validator.update_current_state(self.joint_states)
            return {
                "success": True,
                "joints_moved": moved_joints,
                "message": f"Moved {len(moved_joints)} joints to specified positions"
            }
        else:
            return {"success": False, "message": "Invalid joint data format"}

    async def _handle_get_robot_state(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get robot state service"""
        return {
            "joint_states": self.joint_states.copy(),
            "robot_status": "active" if self.enabled else "disabled",
            "message": "Current robot state retrieved"
        }

    async def _handle_get_joint_limits(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get joint limits service"""
        joint_name = request_data.get("joint_name")
        if joint_name:
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                return {
                    "joint_name": joint_name,
                    "limits": limits,
                    "message": f"Limits for joint {joint_name}"
                }
            else:
                return {"success": False, "message": f"Joint {joint_name} not found"}
        else:
            return {
                "joint_limits": self.joint_limits,
                "message": "All joint limits retrieved"
            }

    async def _handle_check_safety(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle safety check service"""
        # Check for any safety violations
        violations = []

        # Check if joints are within limits
        for joint_name, current_pos in self.joint_states.items():
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                if current_pos < limits["min"] or current_pos > limits["max"]:
                    violations.append(f"Joint {joint_name} at {current_pos}, limits [{limits['min']}, {limits['max']}]")

        if violations:
            return {
                "safety_ok": False,
                "violations": violations,
                "message": f"Safety violations detected: {len(violations)} issues"
            }
        else:
            return {
                "safety_ok": True,
                "violations": [],
                "message": "No safety violations detected"
            }

    async def _handle_emergency_stop(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle emergency stop service"""
        self.enabled = False
        # Reset all joint positions to safe values (could be configurable)
        for joint_name in self.joint_states:
            self.joint_states[joint_name] = 0.0

        return {
            "success": True,
            "message": "Emergency stop activated. Joint control disabled and positions reset."
        }

# Initialize the mock ROS 2 client
ros_client = MockROS2ServiceClient()

# Initialize the NLP command mapper
nlp_mapper = NLPCommandMapper()

@app.get("/")
async def root():
    """Root endpoint for health check"""
    return {
        "message": "ROS 2 AI Agent Bridge API",
        "status": "running",
        "version": "1.0.0"
    }

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "ROS 2 AI Agent Bridge"
    }

@app.post("/translate-command", response_model=ServiceCallResponse)
async def translate_command(command: NaturalLanguageCommand):
    """
    Translate a natural language command to ROS 2 service calls.
    This endpoint uses NLP to convert natural language to appropriate ROS 2 service calls.
    """
    logger.info(f"Received natural language command: {command.command}")

    try:
        # Use the NLP mapper to convert the command
        mapped_command = nlp_mapper.map_command(command.command)

        if not mapped_command:
            SafetyViolationLogger.log_invalid_command(
                command=command.command,
                error_type="NLP_MAPPING_FAILED",
                details={"robot_id": command.robot_id},
                function_name="translate_command"
            )
            return ServiceCallResponse(
                success=False,
                message=f"Could not understand command: {command.command}",
                error="Command not recognized"
            )

        # Log the successful NLP mapping
        SafetyViolationLogger.log_nlp_mapping(
            original_command=command.command,
            mapped_function=mapped_command["function_name"],
            arguments=mapped_command["arguments"]
        )

        # Call the function through the function-call endpoint logic
        function_name = mapped_command["function_name"]
        arguments = mapped_command["arguments"]

        # Use the same logic as the function-call endpoint to execute the mapped command
        if function_name == "enable_joint_control":
            service_info = {
                "service_name": "/joint_control/enable",
                "service_type": "std_srvs/SetBool",
                "request_data": {"data": True}
            }
        elif function_name == "disable_joint_control":
            service_info = {
                "service_name": "/joint_control/disable",
                "service_type": "std_srvs/SetBool",
                "request_data": {"data": False}
            }
        elif function_name == "move_joint_to_position":
            # Validate required arguments
            if "joint_name" not in arguments or "position" not in arguments:
                return ServiceCallResponse(
                    success=False,
                    message="move_joint_to_position requires 'joint_name' and 'position' arguments",
                    error="Missing required arguments"
                )

            service_info = {
                "service_name": "/joint_control/move_to_position",
                "service_type": "sensor_msgs/JointState",
                "request_data": arguments
            }
        elif function_name == "move_multiple_joints":
            # Validate required arguments
            if "joints" not in arguments:
                return ServiceCallResponse(
                    success=False,
                    message="move_multiple_joints requires 'joints' argument",
                    error="Missing required arguments"
                )

            service_info = {
                "service_name": "/joint_control/move_to_position",
                "service_type": "sensor_msgs/JointState",
                "request_data": arguments
            }
        elif function_name == "get_robot_state":
            service_info = {
                "service_name": "/robot_state/get",
                "service_type": "std_srvs/Trigger",
                "request_data": {}
            }
        elif function_name == "execute_preset_pose":
            # Validate required arguments
            if "pose_name" not in arguments:
                return ServiceCallResponse(
                    success=False,
                    message="execute_preset_pose requires 'pose_name' argument",
                    error="Missing required arguments"
                )

            pose_name = arguments["pose_name"]
            if pose_name not in PREDEFINED_POSES:
                return ServiceCallResponse(
                    success=False,
                    message=f"Pose '{pose_name}' not found. Available poses: {list(PREDEFINED_POSES.keys())}",
                    error="Invalid pose name"
                )

            # Convert the pose to joint commands
            joints_data = []
            for joint_name, position in PREDEFINED_POSES[pose_name].items():
                joints_data.append({
                    "joint_name": joint_name,
                    "position": position
                })

            service_info = {
                "service_name": "/joint_control/move_to_position",
                "service_type": "sensor_msgs/JointState",
                "request_data": {"joints": joints_data}
            }
        elif function_name == "get_joint_limits":
            # Validate required arguments
            if "joint_name" not in arguments:
                return ServiceCallResponse(
                    success=False,
                    message="get_joint_limits requires 'joint_name' argument",
                    error="Missing required arguments"
                )

            service_info = {
                "service_name": "/joint_limits/get",
                "service_type": "std_srvs/Trigger",
                "request_data": arguments
            }
        elif function_name == "check_safety_violations":
            service_info = {
                "service_name": "/safety/check",
                "service_type": "std_srvs/Trigger",
                "request_data": {}
            }
        elif function_name == "emergency_stop":
            service_info = {
                "service_name": "/emergency_stop",
                "service_type": "std_srvs/Trigger",
                "request_data": {}
            }
        else:
            return ServiceCallResponse(
                success=False,
                message=f"Function {function_name} not found",
                error="Invalid function name"
            )

        # Call the ROS 2 service
        result = await ros_client.call_service(
            service_info["service_name"],
            service_info["service_type"],
            service_info["request_data"]
        )

        return ServiceCallResponse(
            success=result["success"],
            message=result["message"],
            response_data=result.get("response_data"),
            error=result.get("error")
        )

    except Exception as e:
        logger.error(f"Error processing natural language command: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing command: {str(e)}")

@app.post("/call-service", response_model=ServiceCallResponse)
async def call_service(request: ServiceCallRequest):
    """
    Directly call a ROS 2 service with the given parameters.
    This provides a direct interface to ROS 2 services for more precise control.
    """
    logger.info(f"Direct service call: {request.service_name}")

    try:
        result = await ros_client.call_service(request.service_name, request.service_type, request.request_data)

        return ServiceCallResponse(
            success=result["success"],
            message=result["message"],
            response_data=result.get("response_data"),
            error=result.get("error")
        )

    except Exception as e:
        logger.error(f"Error calling service {request.service_name}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error calling service: {str(e)}")

@app.get("/available-services")
async def available_services():
    """
    List available ROS 2 services that can be called through this API.
    """
    return {
        "services": list(ros_client.services.keys()),
        "message": "Available services that can be called"
    }

@app.post("/function-call")
async def function_call(name: str, arguments: Dict[str, Any]):
    """
    Endpoint for OpenAI function calling.
    This allows the AI agent to call specific functions on the robot.
    """
    logger.info(f"Function call: {name} with arguments: {arguments}")

    try:
        # Determine the appropriate service call based on the function name
        if name == "enable_joint_control":
            service_info = {
                "service_name": "/joint_control/enable",
                "service_type": "std_srvs/SetBool",
                "request_data": {"data": True}
            }
        elif name == "disable_joint_control":
            service_info = {
                "service_name": "/joint_control/disable",
                "service_type": "std_srvs/SetBool",
                "request_data": {"data": False}
            }
        elif name == "move_joint_to_position":
            # Validate required arguments
            if "joint_name" not in arguments or "position" not in arguments:
                SafetyViolationLogger.log_invalid_command(
                    command=f"move_joint_to_position with args: {arguments}",
                    error_type="MISSING_ARGUMENTS",
                    details={"required": ["joint_name", "position"], "provided": list(arguments.keys())},
                    function_name="function_call"
                )
                raise HTTPException(status_code=400, detail="move_joint_to_position requires 'joint_name' and 'position' arguments")

            service_info = {
                "service_name": "/joint_control/move_to_position",
                "service_type": "sensor_msgs/JointState",
                "request_data": arguments
            }
        elif name == "move_multiple_joints":
            # Validate required arguments
            if "joints" not in arguments:
                SafetyViolationLogger.log_invalid_command(
                    command=f"move_multiple_joints with args: {arguments}",
                    error_type="MISSING_ARGUMENTS",
                    details={"required": ["joints"], "provided": list(arguments.keys())},
                    function_name="function_call"
                )
                raise HTTPException(status_code=400, detail="move_multiple_joints requires 'joints' argument")

            service_info = {
                "service_name": "/joint_control/move_to_position",
                "service_type": "sensor_msgs/JointState",
                "request_data": arguments
            }
        elif name == "get_robot_state":
            service_info = {
                "service_name": "/robot_state/get",
                "service_type": "std_srvs/Trigger",
                "request_data": {}
            }
        elif name == "execute_preset_pose":
            # Validate required arguments
            if "pose_name" not in arguments:
                SafetyViolationLogger.log_invalid_command(
                    command=f"execute_preset_pose with args: {arguments}",
                    error_type="MISSING_ARGUMENTS",
                    details={"required": ["pose_name"], "provided": list(arguments.keys())},
                    function_name="function_call"
                )
                raise HTTPException(status_code=400, detail="execute_preset_pose requires 'pose_name' argument")

            pose_name = arguments["pose_name"]
            if pose_name not in PREDEFINED_POSES:
                SafetyViolationLogger.log_invalid_command(
                    command=f"execute_preset_pose for pose: {pose_name}",
                    error_type="INVALID_POSE",
                    details={"available_poses": list(PREDEFINED_POSES.keys()), "requested_pose": pose_name},
                    function_name="function_call"
                )
                raise HTTPException(status_code=400, detail=f"Pose '{pose_name}' not found. Available poses: {list(PREDEFINED_POSES.keys())}")

            # Convert the pose to joint commands
            joints_data = []
            for joint_name, position in PREDEFINED_POSES[pose_name].items():
                joints_data.append({
                    "joint_name": joint_name,
                    "position": position
                })

            service_info = {
                "service_name": "/joint_control/move_to_position",
                "service_type": "sensor_msgs/JointState",
                "request_data": {"joints": joints_data}
            }
        elif name == "get_joint_limits":
            # Validate required arguments
            if "joint_name" not in arguments:
                SafetyViolationLogger.log_invalid_command(
                    command=f"get_joint_limits with args: {arguments}",
                    error_type="MISSING_ARGUMENTS",
                    details={"required": ["joint_name"], "provided": list(arguments.keys())},
                    function_name="function_call"
                )
                raise HTTPException(status_code=400, detail="get_joint_limits requires 'joint_name' argument")

            service_info = {
                "service_name": "/joint_limits/get",
                "service_type": "std_srvs/Trigger",
                "request_data": arguments
            }
        elif name == "check_safety_violations":
            service_info = {
                "service_name": "/safety/check",
                "service_type": "std_srvs/Trigger",
                "request_data": {}
            }
        elif name == "emergency_stop":
            service_info = {
                "service_name": "/emergency_stop",
                "service_type": "std_srvs/Trigger",
                "request_data": {}
            }
        else:
            SafetyViolationLogger.log_invalid_command(
                command=f"function_call for: {name}",
                error_type="UNKNOWN_FUNCTION",
                details={"provided_function": name},
                function_name="function_call"
            )
            raise HTTPException(status_code=404, detail=f"Function {name} not found")

        result = await ros_client.call_service(
            service_info["service_name"],
            service_info["service_type"],
            service_info["request_data"]
        )

        # Log successful command execution
        SafetyViolationLogger.log_command_processed(
            command=name,
            function_name="function_call",
            success=result["success"],
            details={"service_name": service_info["service_name"], "result": result}
        )

        return ServiceCallResponse(
            success=result["success"],
            message=result["message"],
            response_data=result.get("response_data"),
            error=result.get("error")
        )
    except HTTPException:
        # Re-raise HTTP exceptions as they are already handled
        raise
    except Exception as e:
        # Log any other exceptions
        SafetyViolationLogger.log_invalid_command(
            command=f"function_call for: {name}",
            error_type="EXECUTION_ERROR",
            details={"error": str(e), "arguments": arguments},
            function_name="function_call"
        )
        raise HTTPException(status_code=500, detail=f"Error executing function: {str(e)}")

@app.get("/function-definitions")
async def get_function_definitions():
    """
    Return the available function definitions for OpenAI function calling.
    This endpoint provides the JSON schema for all available functions.
    """
    return {"functions": FUNCTION_DEFINITIONS}

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)