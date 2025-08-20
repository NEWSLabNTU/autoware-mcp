"""Factory functions for creating test data and mock responses."""

from typing import Dict, Any, Optional
from datetime import datetime
from autoware_mcp.ad_api_ros2 import (
    OperationModeResponse,
    RouteResponse,
    LocalizationResponse,
    MRMResponse,
)


def create_operation_mode_response(
    success: bool = True,
    current_mode: str = "stop",
    requested_mode: str = "stop",
    message: str = "Success",
    timestamp: Optional[str] = None,
) -> OperationModeResponse:
    """Create an OperationModeResponse for testing."""
    if timestamp is None:
        timestamp = datetime.now().isoformat()
    return OperationModeResponse(
        success=success,
        current_mode=current_mode,
        requested_mode=requested_mode,
        message=message,
        timestamp=timestamp,
    )


def create_route_response(
    success: bool = True,
    message: str = "Route set successfully",
    route_id: Optional[str] = "route_123",
    distance: Optional[float] = 1000.0,
    duration: Optional[float] = 120.0,
) -> RouteResponse:
    """Create a RouteResponse for testing."""
    return RouteResponse(
        success=success,
        message=message,
        route_id=route_id,
        distance=distance,
        duration=duration,
    )


def create_localization_response(
    success: bool = True, message: str = "Localization initialized"
) -> LocalizationResponse:
    """Create a LocalizationResponse for testing."""
    return LocalizationResponse(success=success, message=message)


def create_mrm_response(
    success: bool = True,
    behavior: str = "emergency_stop",
    state: str = "executing",
    message: str = "MRM initiated",
) -> MRMResponse:
    """Create an MRMResponse for testing."""
    return MRMResponse(success=success, behavior=behavior, state=state, message=message)


def create_vehicle_control_response(
    command_type: str = "velocity",
    value: float = 0.0,
    unit: str = "m/s",
    success: bool = True,
) -> Dict[str, Any]:
    """Create a vehicle control command response."""
    return {"success": success, "command": command_type, "value": value, "unit": unit}


def create_system_health_response(
    status: str = "healthy",
    cpu_percent: float = 25.0,
    memory_percent: float = 40.0,
    disk_percent: float = 60.0,
) -> Dict[str, Any]:
    """Create a system health response."""
    return {
        "overall_status": status,
        "timestamp": datetime.now().isoformat(),
        "uptime_seconds": 3600.0,
        "system": {
            "cpu_percent": cpu_percent,
            "memory_percent": memory_percent,
            "disk_percent": disk_percent,
        },
        "ros2": {"available": True, "node_count": 10, "topic_count": 20},
        "autoware": {"workspace": "/opt/autoware", "components_active": 5},
    }


def create_ros2_command_response(
    returncode: int = 0, stdout: str = "", stderr: str = ""
) -> Dict[str, Any]:
    """Create a ROS2 command response."""
    return {"returncode": returncode, "stdout": stdout, "stderr": stderr}
