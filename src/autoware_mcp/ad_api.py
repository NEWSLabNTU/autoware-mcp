"""Autoware AD API integration module."""

import asyncio
import json
from typing import Dict, Any, Optional, List
from enum import Enum
from datetime import datetime

from pydantic import BaseModel, Field
import aiohttp

from .logging import get_logger
from .config import get_config

logger = get_logger(__name__)
config = get_config()


class OperationMode(str, Enum):
    """Vehicle operation modes."""
    STOP = "stop"
    AUTONOMOUS = "autonomous"
    LOCAL = "local"
    REMOTE = "remote"


class OperationModeRequest(BaseModel):
    """Request model for operation mode changes."""
    mode: OperationMode
    transition_time: float = Field(default=10.0, description="Maximum time to wait for transition (seconds)")


class OperationModeResponse(BaseModel):
    """Response model for operation mode operations."""
    success: bool
    current_mode: str
    requested_mode: Optional[str] = None
    message: str
    timestamp: str


class RouteRequest(BaseModel):
    """Request model for route setting."""
    goal_pose: Dict[str, Any] = Field(..., description="Goal pose with position and orientation")
    waypoints: Optional[List[Dict[str, Any]]] = Field(default=None, description="Optional waypoints")
    option: Optional[Dict[str, Any]] = Field(default=None, description="Route options")


class RouteResponse(BaseModel):
    """Response model for route operations."""
    success: bool
    message: str
    route_id: Optional[str] = None
    distance: Optional[float] = None
    duration: Optional[float] = None


class LocalizationRequest(BaseModel):
    """Request model for localization initialization."""
    pose: Dict[str, Any] = Field(..., description="Initial pose with position and orientation")
    pose_with_covariance: Optional[List[float]] = Field(default=None, description="Pose covariance matrix")


class LocalizationResponse(BaseModel):
    """Response model for localization operations."""
    success: bool
    message: str
    initialization_state: str


class MRMRequest(BaseModel):
    """Request model for Minimum Risk Maneuver."""
    behavior: str = Field(..., description="MRM behavior type")
    reason: Optional[str] = Field(default=None, description="Reason for MRM request")


class MRMResponse(BaseModel):
    """Response model for MRM operations."""
    success: bool
    message: str
    current_state: str
    available_behaviors: Optional[List[str]] = None


class VehicleCommand(BaseModel):
    """Model for vehicle control commands."""
    command_type: str = Field(..., description="Type of command: velocity, acceleration, steering, pedals")
    velocity: Optional[float] = Field(default=None, description="Target velocity (m/s)")
    acceleration: Optional[float] = Field(default=None, description="Target acceleration (m/s²)")
    steering_angle: Optional[float] = Field(default=None, description="Steering angle (radians)")
    throttle: Optional[float] = Field(default=None, description="Throttle position (0-1)")
    brake: Optional[float] = Field(default=None, description="Brake position (0-1)")


class AutowareADAPI:
    """Autoware AD API client for MCP integration."""

    def __init__(self, base_url: str = None):
        """Initialize AD API client."""
        self.base_url = base_url or config.autoware.ad_api_url or "http://localhost:8888"
        self.session: Optional[aiohttp.ClientSession] = None
        self._connected = False

    async def connect(self) -> bool:
        """Connect to AD API server."""
        try:
            if self.session is None:
                self.session = aiohttp.ClientSession(
                    timeout=aiohttp.ClientTimeout(total=30)
                )
            
            # Test connection with version endpoint
            async with self.session.get(f"{self.base_url}/api/interface/version") as response:
                if response.status == 200:
                    data = await response.json()
                    logger.info(f"Connected to AD API version: {data}")
                    self._connected = True
                    return True
        except Exception as e:
            logger.error(f"Failed to connect to AD API: {e}")
            self._connected = False
        return False

    async def disconnect(self):
        """Disconnect from AD API server."""
        if self.session:
            await self.session.close()
            self.session = None
        self._connected = False

    async def _call_service(self, endpoint: str, data: Dict[str, Any] = None) -> Dict[str, Any]:
        """Call an AD API service endpoint."""
        if not self._connected:
            if not await self.connect():
                raise ConnectionError("Cannot connect to AD API server")
        
        url = f"{self.base_url}{endpoint}"
        try:
            if data:
                async with self.session.post(url, json=data) as response:
                    result = await response.json()
                    return {"success": response.status == 200, "data": result}
            else:
                async with self.session.get(url) as response:
                    result = await response.json()
                    return {"success": response.status == 200, "data": result}
        except Exception as e:
            logger.error(f"AD API call failed for {endpoint}: {e}")
            return {"success": False, "error": str(e)}

    # Operation Mode Management
    async def change_to_autonomous(self) -> OperationModeResponse:
        """Change operation mode to autonomous."""
        result = await self._call_service("/api/operation_mode/change_to_autonomous")
        return OperationModeResponse(
            success=result["success"],
            current_mode="autonomous" if result["success"] else "unknown",
            requested_mode="autonomous",
            message=result.get("data", {}).get("message", "Mode change requested"),
            timestamp=datetime.now().isoformat()
        )

    async def change_to_stop(self) -> OperationModeResponse:
        """Change operation mode to stop."""
        result = await self._call_service("/api/operation_mode/change_to_stop")
        return OperationModeResponse(
            success=result["success"],
            current_mode="stop" if result["success"] else "unknown",
            requested_mode="stop",
            message=result.get("data", {}).get("message", "Mode change requested"),
            timestamp=datetime.now().isoformat()
        )

    async def change_to_local(self) -> OperationModeResponse:
        """Change operation mode to local manual control."""
        result = await self._call_service("/api/operation_mode/change_to_local")
        return OperationModeResponse(
            success=result["success"],
            current_mode="local" if result["success"] else "unknown",
            requested_mode="local",
            message=result.get("data", {}).get("message", "Mode change requested"),
            timestamp=datetime.now().isoformat()
        )

    async def change_to_remote(self) -> OperationModeResponse:
        """Change operation mode to remote control."""
        result = await self._call_service("/api/operation_mode/change_to_remote")
        return OperationModeResponse(
            success=result["success"],
            current_mode="remote" if result["success"] else "unknown",
            requested_mode="remote",
            message=result.get("data", {}).get("message", "Mode change requested"),
            timestamp=datetime.now().isoformat()
        )

    async def enable_autoware_control(self) -> Dict[str, Any]:
        """Enable Autoware control."""
        return await self._call_service("/api/operation_mode/enable_autoware_control")

    async def disable_autoware_control(self) -> Dict[str, Any]:
        """Disable Autoware control."""
        return await self._call_service("/api/operation_mode/disable_autoware_control")

    async def get_operation_mode_state(self) -> Dict[str, Any]:
        """Get current operation mode state."""
        return await self._call_service("/api/operation_mode/state")

    # Routing and Navigation
    async def set_route(self, goal_pose: Dict[str, Any], option: Optional[Dict] = None) -> RouteResponse:
        """Set a route to a goal pose."""
        data = {"goal": goal_pose}
        if option:
            data["option"] = option
        
        result = await self._call_service("/api/routing/set_route", data)
        return RouteResponse(
            success=result["success"],
            message=result.get("data", {}).get("message", "Route set"),
            route_id=result.get("data", {}).get("route_id"),
            distance=result.get("data", {}).get("distance"),
            duration=result.get("data", {}).get("duration")
        )

    async def set_route_points(self, waypoints: List[Dict[str, Any]], option: Optional[Dict] = None) -> RouteResponse:
        """Set a route with waypoints."""
        data = {"waypoints": waypoints}
        if option:
            data["option"] = option
        
        result = await self._call_service("/api/routing/set_route_points", data)
        return RouteResponse(
            success=result["success"],
            message=result.get("data", {}).get("message", "Route set with waypoints"),
            route_id=result.get("data", {}).get("route_id"),
            distance=result.get("data", {}).get("distance"),
            duration=result.get("data", {}).get("duration")
        )

    async def clear_route(self) -> Dict[str, Any]:
        """Clear the current route."""
        return await self._call_service("/api/routing/clear_route")

    async def get_route_state(self) -> Dict[str, Any]:
        """Get current route state."""
        return await self._call_service("/api/routing/state")

    # Localization
    async def initialize_localization(self, pose: Dict[str, Any], covariance: Optional[List[float]] = None) -> LocalizationResponse:
        """Initialize localization with a pose."""
        data = {"pose": pose}
        if covariance:
            data["pose_with_covariance"] = covariance
        
        result = await self._call_service("/api/localization/initialize", data)
        return LocalizationResponse(
            success=result["success"],
            message=result.get("data", {}).get("message", "Localization initialized"),
            initialization_state=result.get("data", {}).get("state", "unknown")
        )

    async def get_localization_state(self) -> Dict[str, Any]:
        """Get localization initialization state."""
        return await self._call_service("/api/localization/initialization_state")

    # Fail-Safe System
    async def request_mrm(self, behavior: str, reason: Optional[str] = None) -> MRMResponse:
        """Request a Minimum Risk Maneuver."""
        data = {"behavior": behavior}
        if reason:
            data["reason"] = reason
        
        result = await self._call_service("/api/fail_safe/mrm_request/send", data)
        return MRMResponse(
            success=result["success"],
            message=result.get("data", {}).get("message", "MRM requested"),
            current_state=result.get("data", {}).get("state", "unknown"),
            available_behaviors=result.get("data", {}).get("behaviors")
        )

    async def list_mrm_behaviors(self) -> List[str]:
        """List available MRM behaviors."""
        result = await self._call_service("/api/fail_safe/list_mrm_description")
        if result["success"]:
            return result.get("data", {}).get("behaviors", [])
        return []

    async def get_mrm_state(self) -> Dict[str, Any]:
        """Get current MRM state."""
        return await self._call_service("/api/fail_safe/mrm_state")

    # Vehicle Control
    async def send_velocity_command(self, velocity: float) -> Dict[str, Any]:
        """Send velocity control command."""
        data = {"velocity": velocity}
        return await self._call_service("/api/control/command/velocity", data)

    async def send_acceleration_command(self, acceleration: float) -> Dict[str, Any]:
        """Send acceleration control command."""
        data = {"acceleration": acceleration}
        return await self._call_service("/api/control/command/acceleration", data)

    async def send_steering_command(self, steering_angle: float) -> Dict[str, Any]:
        """Send steering control command."""
        data = {"steering_angle": steering_angle}
        return await self._call_service("/api/control/command/steering", data)

    async def send_pedals_command(self, throttle: float = 0.0, brake: float = 0.0) -> Dict[str, Any]:
        """Send pedals control command."""
        data = {"throttle": throttle, "brake": brake}
        return await self._call_service("/api/control/command/pedals", data)

    # Motion Control
    async def accept_start_request(self) -> Dict[str, Any]:
        """Accept motion start request."""
        return await self._call_service("/api/motion/accept_start")

    async def get_motion_state(self) -> Dict[str, Any]:
        """Get current motion state."""
        return await self._call_service("/api/motion/state")

    # Planning Cooperation
    async def get_cooperation_policies(self) -> Dict[str, Any]:
        """Get cooperation policies."""
        return await self._call_service("/api/planning/cooperation/get_policies")

    async def set_cooperation_policies(self, policies: Dict[str, Any]) -> Dict[str, Any]:
        """Set cooperation policies."""
        return await self._call_service("/api/planning/cooperation/set_policies", policies)

    async def send_cooperation_commands(self, commands: Dict[str, Any]) -> Dict[str, Any]:
        """Send cooperation commands."""
        return await self._call_service("/api/planning/cooperation/set_commands", commands)

    # System Monitoring
    async def get_diagnostics_status(self) -> Dict[str, Any]:
        """Get diagnostics status."""
        return await self._call_service("/api/system/diagnostics/status")

    async def reset_diagnostics(self) -> Dict[str, Any]:
        """Reset diagnostics."""
        return await self._call_service("/api/system/diagnostics/reset")

    async def get_system_heartbeat(self) -> Dict[str, Any]:
        """Get system heartbeat."""
        return await self._call_service("/api/system/heartbeat")

    # Vehicle Information
    async def get_vehicle_dimensions(self) -> Dict[str, Any]:
        """Get vehicle dimensions."""
        return await self._call_service("/api/vehicle/dimensions")

    async def get_vehicle_status(self) -> Dict[str, Any]:
        """Get vehicle status."""
        return await self._call_service("/api/vehicle/status")

    async def get_vehicle_kinematics(self) -> Dict[str, Any]:
        """Get vehicle kinematics."""
        return await self._call_service("/api/vehicle/kinematics")


# Global AD API client instance
_ad_api_client: Optional[AutowareADAPI] = None


def get_ad_api() -> AutowareADAPI:
    """Get the global AD API client instance."""
    global _ad_api_client
    if _ad_api_client is None:
        _ad_api_client = AutowareADAPI()
    return _ad_api_client


async def cleanup_ad_api():
    """Cleanup AD API client resources."""
    global _ad_api_client
    if _ad_api_client:
        await _ad_api_client.disconnect()
        _ad_api_client = None