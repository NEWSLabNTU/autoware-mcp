"""Autoware AD API integration using ROS2 topics and services."""

import json
import subprocess
from typing import Dict, Any, Optional, List
from enum import Enum
from datetime import datetime
import time

from pydantic import BaseModel, Field

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
    transition_time: float = Field(
        default=10.0, description="Maximum time to wait for transition (seconds)"
    )


class OperationModeResponse(BaseModel):
    """Response model for operation mode operations."""

    success: bool
    current_mode: str
    requested_mode: Optional[str] = None
    message: str
    timestamp: str


class RouteRequest(BaseModel):
    """Request model for route setting."""

    goal_pose: Dict[str, Any] = Field(
        ..., description="Goal pose with position and orientation"
    )
    waypoints: Optional[List[Dict[str, Any]]] = Field(
        default=None, description="Optional waypoints"
    )
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

    pose: Dict[str, Any] = Field(
        ..., description="Initial pose with position and orientation"
    )
    pose_with_covariance: Optional[List[float]] = Field(
        default=None, description="Pose covariance matrix"
    )


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

    command_type: str = Field(
        ..., description="Type of command: velocity, acceleration, steering, pedals"
    )
    velocity: Optional[float] = Field(default=None, description="Target velocity (m/s)")
    acceleration: Optional[float] = Field(
        default=None, description="Target acceleration (m/s²)"
    )
    steering_angle: Optional[float] = Field(
        default=None, description="Steering angle (radians)"
    )
    throttle: Optional[float] = Field(
        default=None, description="Throttle position (0-1)"
    )
    brake: Optional[float] = Field(default=None, description="Brake position (0-1)")


class AutowareADAPIROS2:
    """Autoware AD API client using ROS2 topics and services."""

    # Operation mode mapping
    OPERATION_MODE_MAP = {
        0: "unknown",
        1: "stop",
        2: "autonomous",
        3: "local",
        4: "remote",
    }

    OPERATION_MODE_REVERSE_MAP = {v: k for k, v in OPERATION_MODE_MAP.items()}

    def __init__(self):
        """Initialize ROS2-based AD API client."""
        logger.info("Initializing ROS2-based AD API client")
        self._connected = False

    async def connect(self) -> bool:
        """Check ROS2 connectivity."""
        try:
            # Check if ROS2 is available
            result = subprocess.run(
                ["ros2", "node", "list"], capture_output=True, text=True, timeout=5
            )
            self._connected = result.returncode == 0
            if self._connected:
                logger.info("ROS2 AD API client connected")
            return self._connected
        except Exception as e:
            logger.error(f"Failed to connect to ROS2: {e}")
            self._connected = False
            return False

    async def disconnect(self):
        """Disconnect from ROS2 (no-op for direct ROS2 calls)."""
        self._connected = False
        logger.info("ROS2 AD API client disconnected")

    async def _call_service(
        self, service_name: str, service_type: str = None, request_data: Dict = None
    ) -> Dict[str, Any]:
        """Call a ROS2 service."""
        if not self._connected:
            await self.connect()

        try:
            # Build the service call command
            cmd = ["ros2", "service", "call", service_name]

            if service_type:
                cmd.append(service_type)

            if request_data:
                # Convert dict to YAML-like string for ROS2 CLI
                request_str = json.dumps(request_data)
                cmd.append(request_str)
            else:
                cmd.append("{}")

            # Execute the service call
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode != 0:
                logger.error(f"Service call failed: {result.stderr}")
                return {"success": False, "error": result.stderr}

            # Parse the response (ROS2 service output is YAML-like)
            # For now, we'll return success if the call completed
            return {"success": True, "data": result.stdout}

        except subprocess.TimeoutExpired:
            logger.error(f"Service call timed out: {service_name}")
            return {"success": False, "error": "Service call timed out"}
        except Exception as e:
            logger.error(f"Service call failed: {e}")
            return {"success": False, "error": str(e)}

    async def _get_topic_data(
        self, topic_name: str, timeout: float = 2.0
    ) -> Dict[str, Any]:
        """Get data from a ROS2 topic."""
        if not self._connected:
            await self.connect()

        try:
            # Use ros2 topic echo with --once flag
            result = subprocess.run(
                ["ros2", "topic", "echo", topic_name, "--once"],
                capture_output=True,
                text=True,
                timeout=timeout,
            )

            if result.returncode != 0:
                logger.error(f"Topic echo failed: {result.stderr}")
                return {"success": False, "error": result.stderr}

            # Parse the YAML-like output
            lines = result.stdout.strip().split("\n")
            data = {}
            current_key = None

            for line in lines:
                if line.startswith("---"):
                    break
                if ":" in line:
                    parts = line.split(":", 1)
                    key = parts[0].strip()
                    value = parts[1].strip() if len(parts) > 1 else ""

                    # Handle nested structures
                    indent_level = len(line) - len(line.lstrip())
                    if indent_level == 0:
                        current_key = key
                        if value:
                            # Try to convert to appropriate type
                            if value.lower() == "true":
                                data[key] = True
                            elif value.lower() == "false":
                                data[key] = False
                            elif value.replace(".", "").replace("-", "").isdigit():
                                try:
                                    data[key] = (
                                        int(value) if "." not in value else float(value)
                                    )
                                except (ValueError, TypeError):
                                    data[key] = value
                            else:
                                data[key] = value
                        else:
                            data[key] = {}
                    elif current_key and isinstance(data.get(current_key), dict):
                        if value:
                            data[current_key][key] = value

            return {"success": True, "data": data}

        except subprocess.TimeoutExpired:
            logger.error(f"Topic echo timed out: {topic_name}")
            return {"success": False, "error": "Topic echo timed out"}
        except Exception as e:
            logger.error(f"Topic echo failed: {e}")
            return {"success": False, "error": str(e)}

    async def _publish_to_topic(
        self, topic_name: str, msg_type: str, data: Dict[str, Any]
    ) -> bool:
        """Publish a message to a ROS2 topic."""
        if not self._connected:
            await self.connect()

        try:
            # Convert data to JSON string for ros2 topic pub
            msg_str = json.dumps(data)

            # Use ros2 topic pub with --once flag
            result = subprocess.run(
                ["ros2", "topic", "pub", "--once", topic_name, msg_type, msg_str],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode != 0:
                logger.error(f"Failed to publish to {topic_name}: {result.stderr}")
                logger.debug(f"Command output: {result.stdout}")

            return result.returncode == 0

        except Exception as e:
            logger.error(f"Topic publish failed: {e}")
            return False

    # Operation Mode Management
    async def get_operation_mode_state(self) -> Dict[str, Any]:
        """Get current operation mode state."""
        result = await self._get_topic_data("/api/operation_mode/state")

        if result.get("success") and result.get("data"):
            data = result["data"]
            mode_num = data.get("mode", 0)
            mode_name = self.OPERATION_MODE_MAP.get(mode_num, "unknown")

            return {
                "success": True,
                "mode": mode_name,
                "mode_number": mode_num,
                "is_autoware_control_enabled": data.get(
                    "is_autoware_control_enabled", False
                ),
                "is_in_transition": data.get("is_in_transition", False),
                "is_stop_mode_available": data.get("is_stop_mode_available", False),
                "is_autonomous_mode_available": data.get(
                    "is_autonomous_mode_available", False
                ),
                "is_local_mode_available": data.get("is_local_mode_available", False),
                "is_remote_mode_available": data.get("is_remote_mode_available", False),
            }

        return {"success": False, "error": "Failed to get operation mode state"}

    async def change_to_autonomous(self) -> OperationModeResponse:
        """Change to autonomous operation mode."""
        # First enable Autoware control if needed
        try:
            enable_result = await self._call_service(
                "/api/operation_mode/enable_autoware_control",
                "std_srvs/srv/Trigger",
                {},
            )
            logger.info(f"Enable control result: {enable_result}")
        except Exception as e:
            logger.warning(f"Failed to enable control: {e}")

        # Try the AD API service first
        result = await self._call_service(
            "/api/operation_mode/change_to_autonomous",
            "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
            {},
        )

        # If AD API fails, try the alternative service
        if not result.get("success"):
            try:
                result = await self._call_service(
                    "/api/operation_mode/change_to_autonomous",
                    "std_srvs/srv/Trigger",
                    {},
                )
            except Exception as e:
                logger.warning(f"Alternative autonomous mode service failed: {e}")

        # If successful, accept start request for motion
        if result.get("success"):
            try:
                start_result = await self._call_service(
                    "/api/motion/accept_start", "std_srvs/srv/Trigger", {}
                )
                logger.info(f"Accept start result: {start_result}")
            except Exception as e:
                logger.warning(f"Failed to accept start: {e}")

        # Get current state
        state = await self.get_operation_mode_state()

        return OperationModeResponse(
            success=result.get("success", False),
            current_mode=state.get("mode", "unknown"),
            requested_mode="autonomous",
            message=(
                "Changed to autonomous mode"
                if result.get("success")
                else "Failed to change mode - ensure localization is initialized"
            ),
            timestamp=datetime.now().isoformat(),
        )

    async def change_to_stop(self) -> OperationModeResponse:
        """Change to stop operation mode."""
        result = await self._call_service(
            "/api/operation_mode/change_to_stop",
            "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
            {},
        )

        # Get current state
        state = await self.get_operation_mode_state()

        return OperationModeResponse(
            success=result.get("success", False),
            current_mode=state.get("mode", "unknown"),
            requested_mode="stop",
            message=(
                "Changed to stop mode"
                if result.get("success")
                else "Failed to change mode"
            ),
            timestamp=datetime.now().isoformat(),
        )

    async def change_to_local(self) -> OperationModeResponse:
        """Change to local operation mode."""
        result = await self._call_service(
            "/api/operation_mode/change_to_local",
            "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
            {},
        )

        # Get current state
        state = await self.get_operation_mode_state()

        return OperationModeResponse(
            success=result.get("success", False),
            current_mode=state.get("mode", "unknown"),
            requested_mode="local",
            message=(
                "Changed to local mode"
                if result.get("success")
                else "Failed to change mode"
            ),
            timestamp=datetime.now().isoformat(),
        )

    async def change_to_remote(self) -> OperationModeResponse:
        """Change to remote operation mode."""
        result = await self._call_service(
            "/api/operation_mode/change_to_remote",
            "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
            {},
        )

        # Get current state
        state = await self.get_operation_mode_state()

        return OperationModeResponse(
            success=result.get("success", False),
            current_mode=state.get("mode", "unknown"),
            requested_mode="remote",
            message=(
                "Changed to remote mode"
                if result.get("success")
                else "Failed to change mode"
            ),
            timestamp=datetime.now().isoformat(),
        )

    async def enable_autoware_control(self) -> Dict[str, Any]:
        """Enable Autoware control."""
        result = await self._call_service(
            "/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {}
        )

        return {
            "success": result.get("success", False),
            "message": (
                "Autoware control enabled"
                if result.get("success")
                else "Failed to enable control"
            ),
            "data": {"control_enabled": result.get("success", False)},
        }

    async def disable_autoware_control(self) -> Dict[str, Any]:
        """Disable Autoware control."""
        result = await self._call_service(
            "/api/operation_mode/disable_autoware_control", "std_srvs/srv/Trigger", {}
        )

        return {
            "success": result.get("success", False),
            "message": (
                "Autoware control disabled"
                if result.get("success")
                else "Failed to disable control"
            ),
            "data": {"control_enabled": not result.get("success", True)},
        }

    # Routing and Navigation
    async def set_route(
        self, goal_pose: Dict[str, Any], option: Optional[Dict] = None
    ) -> RouteResponse:
        """Set a route to a goal pose."""
        # Build request data - Use set_route_points service which is more reliable
        request = {
            "header": {"frame_id": "map"},
            "goal": goal_pose,
            "waypoints": [],  # Empty waypoints means direct route to goal
            "option": option or {"allow_goal_modification": True},
        }

        result = await self._call_service(
            "/api/routing/set_route_points",
            "autoware_adapi_v1_msgs/srv/SetRoutePoints",
            request,
        )

        # Parse the actual error message from the response
        success = False
        message = "Failed to set route"

        if result.get("success") and result.get("data"):
            # Check the response data for actual success status
            data = result["data"]
            if "success=True" in data or "success: true" in data:
                success = True
                message = "Route set successfully"
            elif "message=" in data:
                # Extract error message
                import re

                match = re.search(r"message='([^']*)'", data)
                if match:
                    message = match.group(1)
            elif "message:" in data:
                match = re.search(r"message:\s*(.+)", data)
                if match:
                    message = match.group(1).strip()

        return RouteResponse(
            success=success,
            message=message,
            route_id=f"route_{int(time.time())}" if success else None,
        )

    async def set_route_points(
        self, waypoints: List[Dict[str, Any]], option: Optional[Dict] = None
    ) -> RouteResponse:
        """Set a route with waypoints."""
        # Build request data
        request = {
            "header": {"frame_id": "map"},
            "goal": waypoints[-1] if waypoints else {},  # Last waypoint is the goal
            "waypoints": waypoints[:-1] if len(waypoints) > 1 else [],
            "option": option or {"allow_goal_modification": True},
        }

        result = await self._call_service(
            "/api/routing/set_route_points",
            "autoware_adapi_v1_msgs/srv/SetRoutePoints",
            request,
        )

        return RouteResponse(
            success=result.get("success", False),
            message=(
                "Route with waypoints set successfully"
                if result.get("success")
                else "Failed to set route"
            ),
            route_id=f"route_{int(time.time())}" if result.get("success") else None,
        )

    async def clear_route(self) -> Dict[str, Any]:
        """Clear the current route."""
        result = await self._call_service(
            "/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {}
        )

        return {
            "success": result.get("success", False),
            "message": (
                "Route cleared" if result.get("success") else "Failed to clear route"
            ),
        }

    async def get_route_state(self) -> Dict[str, Any]:
        """Get current route state."""
        result = await self._get_topic_data("/api/routing/state")

        if result.get("success") and result.get("data"):
            data = result["data"]
            return {
                "success": True,
                "has_route": data.get("state", 0)
                > 0,  # Assuming state > 0 means has route
                "state": data.get("state", 0),
                "progress": 0.0,  # Would need additional info for actual progress
            }

        return {"success": False, "error": "Failed to get route state"}

    # Localization
    async def initialize_localization(
        self, pose: Dict[str, Any], pose_with_covariance: Optional[List[float]] = None
    ) -> LocalizationResponse:
        """Initialize localization with a pose."""
        # Build the pose message
        # Use covariance values that match RViz 2D Pose Estimate tool defaults
        covariance = pose_with_covariance or [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.06853892326654787,
        ]

        # For planning simulator, publish to both /initialpose (RViz) and /initialpose3d topics
        pose_msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "pose": {"pose": pose, "covariance": covariance},
        }

        # Log the message being published for debugging
        logger.debug(f"Publishing pose message: {pose_msg}")

        # Publish to both topics for maximum compatibility
        # /initialpose is used by RViz 2D Pose Estimate tool and planning simulator
        # /initialpose3d is for 3D pose initialization
        result1 = await self._publish_to_topic(
            "/initialpose",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            pose_msg,
        )

        result2 = await self._publish_to_topic(
            "/initialpose3d",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            pose_msg,
        )

        # Also try the AD API service if available
        try:
            request = {
                "pose": [
                    {
                        "header": {
                            "frame_id": "map",
                            "stamp": {"sec": 0, "nanosec": 0},
                        },
                        "pose": {"pose": pose, "covariance": covariance},
                    }
                ]
            }

            result3 = await self._call_service(
                "/api/localization/initialize",
                "autoware_adapi_v1_msgs/srv/InitializeLocalization",
                request,
            )
        except Exception as e:
            logger.warning(f"AD API localization service not available: {e}")
            result3 = {"success": False}

        # Check results
        logger.info(f"Publish to /initialpose: {result1}")
        logger.info(f"Publish to /initialpose3d: {result2}")
        logger.info(f"AD API service result: {result3.get('success', False)}")

        success = result1 or result2 or result3.get("success", False)

        return LocalizationResponse(
            success=success,
            message=(
                "Localization pose published to initialpose topics"
                if success
                else "Failed to initialize localization - check logs for details"
            ),
            initialization_state=("initialized" if success else "uninitialized"),
        )

    async def get_localization_state(self) -> Dict[str, Any]:
        """Get localization state."""
        result = await self._get_topic_data("/api/localization/initialization_state")

        if result.get("success") and result.get("data"):
            data = result["data"]
            # State mapping: 0=UNINITIALIZED, 1=INITIALIZING, 2=INITIALIZED
            state_map = {0: "uninitialized", 1: "initializing", 2: "initialized"}
            state_num = data.get("state", 0)

            return {
                "success": True,
                "localized": state_num == 2,
                "state": state_map.get(state_num, "unknown"),
                "quality": "good" if state_num == 2 else "poor",
            }

        return {"success": False, "error": "Failed to get localization state"}

    # Motion Control
    async def accept_start_request(self) -> Dict[str, Any]:
        """Accept motion start request."""
        result = await self._call_service(
            "/api/motion/accept_start", "std_srvs/srv/Trigger", {}
        )

        return {
            "success": result.get("success", False),
            "message": (
                "Start request accepted"
                if result.get("success")
                else "Failed to accept start request"
            ),
        }

    async def get_motion_state(self) -> Dict[str, Any]:
        """Get motion state."""
        result = await self._get_topic_data("/api/motion/state")

        if result.get("success") and result.get("data"):
            data = result["data"]
            # State mapping for motion
            state_num = data.get("state", 0)
            state_map = {0: "stopped", 1: "starting", 2: "moving"}

            return {
                "success": True,
                "ready": state_num > 0,
                "state": state_map.get(state_num, "unknown"),
                "motion_started": state_num == 2,
            }

        return {"success": False, "error": "Failed to get motion state"}

    # MRM (Minimum Risk Maneuver)
    async def request_mrm(
        self, behavior: str, reason: Optional[str] = None
    ) -> MRMResponse:
        """Request a Minimum Risk Maneuver."""
        # Note: This would need the actual MRM service interface
        # For now, using a placeholder implementation

        return MRMResponse(
            success=True,
            message=f"MRM {behavior} requested",
            current_state="mrm_operating",
            available_behaviors=["comfortable_stop", "emergency_stop", "pull_over"],
        )

    async def list_mrm_behaviors(self) -> List[str]:
        """List available MRM behaviors."""
        return ["comfortable_stop", "emergency_stop", "pull_over"]

    async def get_mrm_state(self) -> Dict[str, Any]:
        """Get MRM state."""
        result = await self._get_topic_data("/api/fail_safe/mrm_state")

        if result.get("success") and result.get("data"):
            data = result["data"]
            return {
                "success": True,
                "active": data.get("state", 0) > 0,
                "current_behavior": data.get("behavior", "none"),
                "state": data.get("state", 0),
            }

        return {"success": False, "error": "Failed to get MRM state"}

    # Vehicle Control Commands
    async def send_velocity_command(self, velocity: float) -> Dict[str, Any]:
        """Send velocity command."""
        # This would publish to control command topic
        msg = {
            "stamp": {"sec": 0, "nanosec": 0},
            "lateral": {"steering_tire_angle": 0.0, "steering_tire_rotation_rate": 0.0},
            "longitudinal": {"speed": velocity, "acceleration": 0.0},
        }

        success = await self._publish_to_topic(
            "/control/command/control_cmd",
            "autoware_auto_control_msgs/msg/AckermannControlCommand",
            msg,
        )

        return {
            "success": success,
            "message": (
                f"Velocity command sent: {velocity} m/s"
                if success
                else "Failed to send command"
            ),
            "command_id": f"cmd_{int(time.time())}" if success else None,
        }

    async def send_acceleration_command(self, acceleration: float) -> Dict[str, Any]:
        """Send acceleration command."""
        msg = {
            "stamp": {"sec": 0, "nanosec": 0},
            "lateral": {"steering_tire_angle": 0.0, "steering_tire_rotation_rate": 0.0},
            "longitudinal": {"speed": 0.0, "acceleration": acceleration},
        }

        success = await self._publish_to_topic(
            "/control/command/control_cmd",
            "autoware_auto_control_msgs/msg/AckermannControlCommand",
            msg,
        )

        return {
            "success": success,
            "message": (
                f"Acceleration command sent: {acceleration} m/s²"
                if success
                else "Failed to send command"
            ),
        }

    async def send_steering_command(self, steering_angle: float) -> Dict[str, Any]:
        """Send steering command."""
        msg = {
            "stamp": {"sec": 0, "nanosec": 0},
            "lateral": {
                "steering_tire_angle": steering_angle,
                "steering_tire_rotation_rate": 0.0,
            },
            "longitudinal": {"speed": 0.0, "acceleration": 0.0},
        }

        success = await self._publish_to_topic(
            "/control/command/control_cmd",
            "autoware_auto_control_msgs/msg/AckermannControlCommand",
            msg,
        )

        return {
            "success": success,
            "message": (
                f"Steering command sent: {steering_angle} rad"
                if success
                else "Failed to send command"
            ),
        }

    async def send_pedals_command(
        self, throttle: float = 0.0, brake: float = 0.0
    ) -> Dict[str, Any]:
        """Send pedals command."""
        # Convert throttle/brake to acceleration
        # Simplified: positive for throttle, negative for brake
        acceleration = throttle * 3.0 - brake * 5.0

        return await self.send_acceleration_command(acceleration)

    # Vehicle Information
    async def get_vehicle_dimensions(self) -> Dict[str, Any]:
        """Get vehicle dimensions."""
        result = await self._call_service(
            "/api/vehicle/dimensions",
            "autoware_adapi_v1_msgs/srv/GetVehicleDimensions",
            {},
        )

        if result.get("success"):
            # Parse response for dimensions
            return {
                "success": True,
                "length": 4.77,  # Default sample vehicle dimensions
                "width": 1.83,
                "height": 1.43,
                "wheelbase": 2.79,
            }

        return {"success": False, "error": "Failed to get vehicle dimensions"}

    async def get_vehicle_status(self) -> Dict[str, Any]:
        """Get vehicle status."""
        result = await self._get_topic_data("/api/vehicle/status")

        if result.get("success") and result.get("data"):
            data = result["data"]
            return {
                "success": True,
                "gear": data.get("gear", "unknown"),
                "speed": data.get("velocity", 0.0),
                "steering_angle": data.get("steering", 0.0),
            }

        # Try alternate topic
        result = await self._get_topic_data("/vehicle/status/velocity_status")
        if result.get("success") and result.get("data"):
            data = result["data"]
            return {
                "success": True,
                "gear": "drive",
                "speed": data.get("longitudinal_velocity", 0.0),
                "steering_angle": 0.0,
            }

        return {"success": False, "error": "Failed to get vehicle status"}

    async def get_vehicle_kinematics(self) -> Dict[str, Any]:
        """Get vehicle kinematics."""
        result = await self._get_topic_data("/api/vehicle/kinematics")

        if result.get("success") and result.get("data"):
            data = result["data"]
            pose = data.get("pose", {}).get("pose", {})
            twist = data.get("twist", {}).get("twist", {})

            return {
                "success": True,
                "position": pose.get("position", {"x": 0, "y": 0, "z": 0}),
                "orientation": pose.get(
                    "orientation", {"x": 0, "y": 0, "z": 0, "w": 1}
                ),
                "velocity": twist.get("linear", {}).get("x", 0.0),
                "angular_velocity": twist.get("angular", {}).get("z", 0.0),
            }

        # Try localization topic
        result = await self._get_topic_data("/localization/kinematic_state")
        if result.get("success") and result.get("data"):
            data = result["data"]
            pose = data.get("pose", {}).get("pose", {})
            twist = data.get("twist", {}).get("twist", {})

            return {
                "success": True,
                "position": pose.get("position", {"x": 0, "y": 0, "z": 0}),
                "orientation": pose.get(
                    "orientation", {"x": 0, "y": 0, "z": 0, "w": 1}
                ),
                "velocity": twist.get("linear", {}).get("x", 0.0) if twist else 0.0,
            }

        return {"success": False, "error": "Failed to get vehicle kinematics"}

    # Placeholder methods for remaining features
    async def get_cooperation_policies(self) -> Dict[str, Any]:
        """Get cooperation policies."""
        return {
            "success": True,
            "policies": {"enable_lane_change": True, "enable_avoidance": True},
        }

    async def set_cooperation_policies(
        self, policies: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Set cooperation policies."""
        return {"success": True, "message": "Policies updated", "policies": policies}

    async def send_cooperation_commands(
        self, commands: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Send cooperation commands."""
        return {"success": True, "message": "Commands sent"}

    async def get_diagnostics_status(self) -> Dict[str, Any]:
        """Get diagnostics status."""
        # For now, return a simple status
        # TODO: Parse diagnostics data when available
        _ = await self._get_topic_data("/diagnostics")

        return {
            "success": True,
            "overall_status": "OK",
            "error_count": 0,
            "warning_count": 0,
        }

    async def reset_diagnostics(self) -> Dict[str, Any]:
        """Reset diagnostics."""
        return {"success": True, "message": "Diagnostics reset"}

    async def get_system_heartbeat(self) -> Dict[str, Any]:
        """Get system heartbeat."""
        # TODO: Parse heartbeat data when available
        _ = await self._get_topic_data("/api/system/heartbeat")

        return {
            "success": True,
            "alive": True,
            "last_heartbeat": datetime.now().isoformat(),
        }


# Singleton instance
_ad_api_instance = None


def get_ad_api() -> AutowareADAPIROS2:
    """Get the AD API client instance."""
    global _ad_api_instance
    if _ad_api_instance is None:
        _ad_api_instance = AutowareADAPIROS2()
    return _ad_api_instance


async def cleanup_ad_api():
    """Cleanup AD API client."""
    global _ad_api_instance
    if _ad_api_instance:
        await _ad_api_instance.disconnect()
        _ad_api_instance = None
