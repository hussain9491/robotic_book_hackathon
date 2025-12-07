"""
Logging utilities for the ROS 2 AI Agent Bridge
This module provides structured logging for safety violations and invalid commands
"""

import logging
from datetime import datetime
from typing import Dict, Any, Optional
import json

# Create a custom logger for the bridge
bridge_logger = logging.getLogger("ros2_ai_bridge")
bridge_logger.setLevel(logging.INFO)

# Create file handler for detailed logs
file_handler = logging.FileHandler("ros2_ai_bridge.log")
file_handler.setLevel(logging.INFO)

# Create console handler for important messages
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.WARNING)

# Create formatter
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

# Add handlers to the logger
bridge_logger.addHandler(file_handler)
bridge_logger.addHandler(console_handler)

class SafetyViolationLogger:
    """Structured logger for safety violations and invalid commands"""

    @staticmethod
    def log_safety_violation(
        violation_type: str,
        details: Dict[str, Any],
        command: Optional[str] = None,
        function_name: Optional[str] = None
    ):
        """Log a safety violation with structured details"""
        violation_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": "SAFETY_VIOLATION",
            "violation_type": violation_type,
            "command": command,
            "function_name": function_name,
            "details": details
        }

        bridge_logger.warning(f"SAFETY VIOLATION: {json.dumps(violation_data, indent=2)}")

    @staticmethod
    def log_invalid_command(
        command: str,
        error_type: str,
        details: Dict[str, Any],
        function_name: Optional[str] = None
    ):
        """Log an invalid command with structured details"""
        invalid_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": "INVALID_COMMAND",
            "command": command,
            "error_type": error_type,
            "function_name": function_name,
            "details": details
        }

        bridge_logger.warning(f"INVALID COMMAND: {json.dumps(invalid_data, indent=2)}")

    @staticmethod
    def log_command_processed(
        command: str,
        function_name: str,
        success: bool,
        details: Optional[Dict[str, Any]] = None
    ):
        """Log a processed command"""
        status = "SUCCESS" if success else "FAILED"
        processed_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": "COMMAND_PROCESSED",
            "command": command,
            "function_name": function_name,
            "status": status,
            "details": details or {}
        }

        log_level = logging.INFO if success else logging.ERROR
        bridge_logger.log(log_level, f"COMMAND PROCESSED: {status}: {json.dumps(processed_data, indent=2)}")

    @staticmethod
    def log_nlp_mapping(
        original_command: str,
        mapped_function: str,
        arguments: Dict[str, Any],
        confidence: Optional[float] = None
    ):
        """Log NLP command mapping"""
        mapping_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": "NLP_MAPPING",
            "original_command": original_command,
            "mapped_function": mapped_function,
            "arguments": arguments,
            "confidence": confidence
        }

        bridge_logger.info(f"NLP MAPPING: {json.dumps(mapping_data, indent=2)}")