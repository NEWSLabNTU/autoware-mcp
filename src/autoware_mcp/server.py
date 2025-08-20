"""Main MCP server implementation using FastMCP."""

import asyncio
import os
from typing import Dict, Any, List, Optional
from pathlib import Path
from datetime import datetime

from fastmcp import FastMCP
from pydantic import BaseModel, Field

from .config import get_config
from .ros2_manager import get_ros2_manager
from .health import get_monitor
from .logging import setup_logging
from .ad_api_ros2 import (
    get_ad_api,
    cleanup_ad_api,
    OperationMode,
    OperationModeRequest,
    OperationModeResponse,
    RouteRequest,
    RouteResponse,
    LocalizationRequest,
    LocalizationResponse,
    MRMRequest,
    MRMResponse,
)

# Initialize FastMCP server
mcp = FastMCP("Autoware MCP Server")
mcp.description = "Model Context Protocol server for Autoware autonomous driving stack"

# Initialize logging
config = get_config()
logger = setup_logging(
    log_level=config.server.log_level,
    log_file=Path.home() / ".autoware-mcp" / "server.log",
)

# Get global instances
ros2_manager = get_ros2_manager()
monitor = get_monitor()
ad_api = get_ad_api()


# Pydantic models for API
class HealthResponse(BaseModel):
    """Health check response model."""

    status: str
    timestamp: str
    uptime_seconds: float
    system: Dict[str, Any]
    ros2: Dict[str, Any]
    autoware: Dict[str, Any]


class NodeInfo(BaseModel):
    """ROS2 node information."""

    name: str
    namespace: str = "/"
    state: str = "unknown"


class TopicInfo(BaseModel):
    """ROS2 topic information."""

    name: str
    message_type: str = "unknown"
    publishers: int = 0
    subscribers: int = 0


# Health and status endpoints
@mcp.tool()
async def health_check() -> HealthResponse:
    """
    Get complete health status of the Autoware MCP server.

    Returns comprehensive health information including:
    - System resources (CPU, memory, disk)
    - ROS2 status and active nodes/topics
    - Autoware workspace configuration
    - Overall health status
    """
    logger.info("Health check requested")
    health_data = await monitor.get_complete_health_status()
    # Map overall_status to status field expected by HealthResponse
    health_data["status"] = health_data.pop("overall_status", "unknown")
    return HealthResponse(**health_data)


@mcp.tool()
async def get_system_status() -> Dict[str, Any]:
    """
    Get current system resource usage and status.

    Returns:
    - CPU usage and frequency
    - Memory usage
    - Disk usage
    - Network interfaces
    - Process counts
    """
    logger.info("System status requested")
    return await monitor.get_system_info()


# ROS2 operations
@mcp.tool()
async def list_ros2_nodes() -> List[str]:
    """
    List all active ROS2 nodes in the system.

    Returns list of node names with their namespaces.
    """
    logger.info("Listing ROS2 nodes")
    await ros2_manager.initialize()
    nodes = await ros2_manager.list_nodes()
    logger.info(f"Found {len(nodes)} active nodes")
    return nodes


@mcp.tool()
async def list_ros2_topics() -> List[str]:
    """
    List all active ROS2 topics in the system.

    Returns list of topic names.
    """
    logger.info("Listing ROS2 topics")
    await ros2_manager.initialize()
    topics = await ros2_manager.list_topics()
    logger.info(f"Found {len(topics)} active topics")
    return topics


@mcp.tool()
async def get_node_info(
    node_name: str = Field(description="Name of the ROS2 node"),
) -> Dict[str, Any]:
    """
    Get detailed information about a specific ROS2 node.

    Args:
        node_name: Full name of the node (including namespace)

    Returns:
        Node information including publishers, subscribers, services, and parameters.
    """
    logger.info(f"Getting info for node: {node_name}")
    await ros2_manager.initialize()
    info = await ros2_manager.get_node_info(node_name)
    return info


# Autoware observation and interaction
@mcp.tool()
async def check_autoware_status() -> Dict[str, Any]:
    """
    Check the current status of running Autoware components.

    Analyzes the ROS2 graph to identify:
    - Active Autoware nodes by category (perception, planning, control, etc.)
    - Autoware-specific topics
    - Component health status

    Returns detailed status of running Autoware components.
    """
    logger.info("Checking Autoware status")
    await ros2_manager.initialize()
    status = await ros2_manager.check_autoware_nodes()
    logger.info(f"Autoware running: {status['autoware_running']}")
    return status


@mcp.tool()
async def list_ros2_services() -> List[str]:
    """
    List all active ROS2 services in the system.

    Returns list of service names that can be called.
    """
    logger.info("Listing ROS2 services")
    await ros2_manager.initialize()
    services = await ros2_manager.list_services()
    logger.info(f"Found {len(services)} active services")
    return services


@mcp.tool()
async def get_topic_info(
    topic_name: str = Field(description="Name of the ROS2 topic"),
) -> Dict[str, Any]:
    """
    Get detailed information about a specific ROS2 topic.

    Args:
        topic_name: Name of the topic (e.g., "/tf", "/map")

    Returns:
        Topic information including message type, publisher count, subscriber count.
    """
    logger.info(f"Getting info for topic: {topic_name}")
    await ros2_manager.initialize()
    info = await ros2_manager.get_topic_info(topic_name)
    return info


@mcp.tool()
async def get_topic_frequency(
    topic_name: str = Field(description="Name of the ROS2 topic"),
    duration: float = Field(
        default=5.0, description="Duration to measure frequency (seconds)"
    ),
) -> Dict[str, Any]:
    """
    Measure the publishing frequency of a ROS2 topic.

    Args:
        topic_name: Name of the topic to measure
        duration: How long to measure (default: 5 seconds)

    Returns:
        Frequency information including average Hz.
    """
    logger.info(f"Measuring frequency for topic: {topic_name}")
    await ros2_manager.initialize()
    freq_info = await ros2_manager.get_topic_hz(topic_name, duration)
    return freq_info


@mcp.tool()
async def echo_topic_messages(
    topic_name: str = Field(description="Name of the ROS2 topic"),
    count: int = Field(default=1, description="Number of messages to capture"),
) -> Dict[str, Any]:
    """
    Capture and return messages from a ROS2 topic.

    Args:
        topic_name: Name of the topic to echo
        count: Number of messages to capture (default: 1)

    Returns:
        Captured messages from the topic.
    """
    logger.info(f"Echoing {count} messages from topic: {topic_name}")
    await ros2_manager.initialize()
    messages = await ros2_manager.echo_topic(topic_name, count)
    return messages


# Generic ROS2 interaction tools
@mcp.tool()
async def call_ros2_service(
    service_name: str = Field(
        ..., description="Full service name (e.g., /api/routing/clear_route)"
    ),
    service_type: str = Field(
        ..., description="Service type (e.g., std_srvs/srv/Trigger)"
    ),
    request: Dict[str, Any] = Field(
        default={}, description="Service request data as JSON"
    ),
) -> Dict[str, Any]:
    """
    Call any ROS2 service directly.

    This is a powerful generic tool that can call any ROS2 service in the system.
    Use 'list_ros2_services' to discover available services.

    Examples:
    - Clear route: call_ros2_service("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {})
    - Enable control: call_ros2_service("/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {})
    - Accept start: call_ros2_service("/api/motion/accept_start", "std_srvs/srv/Trigger", {})

    Returns:
    - success: Whether the service call succeeded
    - response: The service response data
    - error: Error message if call failed
    """
    logger.info(f"Calling ROS2 service: {service_name}")

    try:
        result = await ros2_manager.call_service(service_name, service_type, request)
        return {
            "success": result.get("success", False),
            "response": result.get("data", {}),
            "error": result.get("error", None),
        }
    except Exception as e:
        logger.error(f"Service call failed: {e}")
        return {"success": False, "response": {}, "error": str(e)}


@mcp.tool()
async def publish_to_topic(
    topic_name: str = Field(..., description="Topic name to publish to"),
    message_type: str = Field(
        ..., description="Message type (e.g., geometry_msgs/msg/Twist)"
    ),
    message: Dict[str, Any] = Field(..., description="Message data to publish as JSON"),
) -> Dict[str, Any]:
    """
    Publish a message to any ROS2 topic.

    This tool allows publishing to any ROS2 topic with the appropriate message type.
    Use 'get_topic_info' to discover the message type for a topic.

    Examples:
    - Publish twist: publish_to_topic("/cmd_vel", "geometry_msgs/msg/Twist", {"linear": {"x": 1.0}, "angular": {"z": 0.5}})
    - Publish initial pose: publish_to_topic("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", {...})

    Returns:
    - success: Whether the publish succeeded
    - error: Error message if publish failed
    """
    logger.info(f"Publishing to topic: {topic_name}")

    try:
        success = await ros2_manager.publish_to_topic(topic_name, message_type, message)
        return {
            "success": success,
            "error": None if success else "Failed to publish message",
        }
    except Exception as e:
        logger.error(f"Topic publish failed: {e}")
        return {"success": False, "error": str(e)}


# Configuration management
@mcp.tool()
async def get_configuration() -> Dict[str, Any]:
    """
    Get current MCP server configuration.

    Returns all configuration parameters including:
    - Autoware workspace path
    - ROS distribution
    - Server settings
    """
    logger.info("Configuration requested")
    return config.dict()


@mcp.tool()
async def verify_ros2_environment() -> Dict[str, Any]:
    """
    Verify ROS2 environment is properly set up.

    The MCP server expects users to source their ROS2/Autoware environment
    before starting the server. This tool checks if the environment is ready.

    Returns:
    - ROS2 availability and version
    - Environment variables status
    - Whether system is ready for MCP operations
    """
    logger.info("Verifying ROS2 environment")

    result = {
        "valid": False,
        "checks": {},
        "errors": [],
        "warnings": [],
    }

    # Check critical environment variables
    env_vars = {
        "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
        "ROS_VERSION": os.environ.get("ROS_VERSION"),
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
        "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH"),
    }

    result["environment"] = {k: v for k, v in env_vars.items() if v is not None}

    if not env_vars["ROS_DISTRO"]:
        result["errors"].append(
            "ROS_DISTRO not set. Please source your ROS2/Autoware setup.bash first."
        )

    if not env_vars["AMENT_PREFIX_PATH"]:
        result["warnings"].append(
            "AMENT_PREFIX_PATH not set. ROS2 environment may not be properly sourced."
        )

    # Check ROS2 command availability
    try:
        ros_result = await ros2_manager.run_command(["ros2", "--version"])
        result["checks"]["ros2_available"] = ros_result["returncode"] == 0
        if ros_result["returncode"] == 0:
            result["ros2_version"] = ros_result["stdout"].strip()
    except Exception as e:
        result["checks"]["ros2_available"] = False
        result["errors"].append(f"ROS2 command check failed: {str(e)}")

    # Check if we can list topics (ultimate test of ROS2 working)
    try:
        topics_result = await ros2_manager.run_command(
            ["ros2", "topic", "list", "--spin-time", "0"], timeout=5.0
        )
        result["checks"]["can_list_topics"] = topics_result["returncode"] == 0
    except Exception:
        result["checks"]["can_list_topics"] = False

    # Determine overall validity
    result["valid"] = (
        result["checks"].get("ros2_available", False) and len(result["errors"]) == 0
    )

    logger.info(
        f"Environment verification result: {'valid' if result['valid'] else 'invalid'}"
    )
    return result


# =============================================================================
# Autoware AD API Tools
# =============================================================================


# Operation Mode Management
@mcp.tool()
async def set_operation_mode(request: OperationModeRequest) -> OperationModeResponse:
    """
    Change vehicle operation mode.

    Modes:
    - stop: Stop vehicle operation
    - autonomous: Switch to autonomous driving
    - local: Switch to local manual control
    - remote: Switch to remote control

    Args:
        request: Operation mode request with mode and optional transition time

    Returns:
        Operation mode response with success status and current mode
    """
    logger.info(f"Setting operation mode to: {request.mode}")

    if request.mode == OperationMode.AUTONOMOUS:
        return await ad_api.change_to_autonomous()
    elif request.mode == OperationMode.STOP:
        return await ad_api.change_to_stop()
    elif request.mode == OperationMode.LOCAL:
        return await ad_api.change_to_local()
    elif request.mode == OperationMode.REMOTE:
        return await ad_api.change_to_remote()
    else:
        return OperationModeResponse(
            success=False,
            current_mode="unknown",
            requested_mode=request.mode,
            message=f"Unknown operation mode: {request.mode}",
            timestamp=datetime.now().isoformat(),
        )


@mcp.tool()
async def monitor_operation_mode() -> Dict[str, Any]:
    """
    Monitor current operation mode state.

    Returns:
        Current operation mode and control state
    """
    logger.info("Getting operation mode state")
    return await ad_api.get_operation_mode_state()


# Routing and Navigation
@mcp.tool()
async def set_route(request: RouteRequest) -> RouteResponse:
    """
    Set a route to a goal pose.

    Args:
        request: Route request with goal pose and optional waypoints

    Returns:
        Route response with success status and route information
    """
    logger.info("Setting route")

    if request.waypoints:
        return await ad_api.set_route_points(request.waypoints, request.option)
    else:
        return await ad_api.set_route(request.goal_pose, request.option)


@mcp.tool()
async def set_route_points(
    waypoints: List[Dict[str, Any]] = Field(..., description="Waypoints for the route"),
    option: Optional[Dict[str, Any]] = Field(default=None, description="Route options"),
) -> RouteResponse:
    """
    Set a route with specific waypoints.

    Args:
        waypoints: List of waypoints with position and orientation
        option: Optional route options

    Returns:
        Route response with success status and route information
    """
    logger.info(f"Setting route with {len(waypoints)} waypoints")
    return await ad_api.set_route_points(waypoints, option)


@mcp.tool()
async def get_current_route() -> Dict[str, Any]:
    """
    Get current route state.

    Returns:
        Current route information including waypoints and progress
    """
    logger.info("Getting current route")
    return await ad_api.get_route_state()


# Localization
@mcp.tool()
async def initialize_localization(request: LocalizationRequest) -> LocalizationResponse:
    """
    Initialize localization with a pose.

    Args:
        request: Localization request with initial pose

    Returns:
        Localization response with success status
    """
    logger.info("Initializing localization")
    return await ad_api.initialize_localization(
        request.pose, request.pose_with_covariance
    )


@mcp.tool()
async def monitor_localization_state() -> Dict[str, Any]:
    """
    Monitor localization initialization state.

    Returns:
        Current localization state and quality metrics
    """
    logger.info("Getting localization state")
    return await ad_api.get_localization_state()


# Fail-Safe System
@mcp.tool()
async def request_mrm(request: MRMRequest) -> MRMResponse:
    """
    Request a Minimum Risk Maneuver (MRM).

    Args:
        request: MRM request with behavior type and optional reason

    Returns:
        MRM response with success status and current state
    """
    logger.info(f"Requesting MRM: {request.behavior}")
    return await ad_api.request_mrm(request.behavior, request.reason)


@mcp.tool()
async def list_mrm_behaviors() -> List[str]:
    """
    List available MRM behaviors.

    Returns:
        List of available MRM behavior types
    """
    logger.info("Listing MRM behaviors")
    return await ad_api.list_mrm_behaviors()


@mcp.tool()
async def monitor_mrm_state() -> Dict[str, Any]:
    """
    Monitor current MRM state.

    Returns:
        Current MRM state and active behaviors
    """
    logger.info("Getting MRM state")
    return await ad_api.get_mrm_state()


# Vehicle Control Commands
@mcp.tool()
async def send_velocity_command(
    velocity: float = Field(..., description="Target velocity in m/s", ge=-10, le=50),
) -> Dict[str, Any]:
    """
    Send velocity control command.

    Args:
        velocity: Target velocity in m/s

    Returns:
        Command acknowledgment
    """
    logger.info(f"Sending velocity command: {velocity} m/s")
    return await ad_api.send_velocity_command(velocity)


@mcp.tool()
async def send_acceleration_command(
    acceleration: float = Field(
        ..., description="Target acceleration in m/s²", ge=-5, le=3
    ),
) -> Dict[str, Any]:
    """
    Send acceleration control command.

    Args:
        acceleration: Target acceleration in m/s²

    Returns:
        Command acknowledgment
    """
    logger.info(f"Sending acceleration command: {acceleration} m/s²")
    return await ad_api.send_acceleration_command(acceleration)


@mcp.tool()
async def send_steering_command(
    steering_angle: float = Field(
        ..., description="Steering angle in radians", ge=-0.7, le=0.7
    ),
) -> Dict[str, Any]:
    """
    Send steering control command.

    Args:
        steering_angle: Steering angle in radians

    Returns:
        Command acknowledgment
    """
    logger.info(f"Sending steering command: {steering_angle} rad")
    return await ad_api.send_steering_command(steering_angle)


@mcp.tool()
async def send_pedals_command(
    throttle: float = Field(
        default=0.0, description="Throttle position (0-1)", ge=0, le=1
    ),
    brake: float = Field(default=0.0, description="Brake position (0-1)", ge=0, le=1),
) -> Dict[str, Any]:
    """
    Send pedals control command.

    Args:
        throttle: Throttle position (0-1)
        brake: Brake position (0-1)

    Returns:
        Command acknowledgment
    """
    logger.info(f"Sending pedals command: throttle={throttle}, brake={brake}")
    return await ad_api.send_pedals_command(throttle, brake)


# Motion Control


@mcp.tool()
async def monitor_motion_state() -> Dict[str, Any]:
    """
    Monitor current motion state.

    Returns:
        Current motion state and readiness
    """
    logger.info("Getting motion state")
    return await ad_api.get_motion_state()


# Planning Cooperation
@mcp.tool()
async def get_cooperation_policies() -> Dict[str, Any]:
    """
    Get current cooperation policies.

    Returns:
        Active cooperation policies
    """
    logger.info("Getting cooperation policies")
    return await ad_api.get_cooperation_policies()


@mcp.tool()
async def set_cooperation_policies(
    policies: Dict[str, Any] = Field(..., description="Cooperation policies to set"),
) -> Dict[str, Any]:
    """
    Set cooperation policies.

    Args:
        policies: Cooperation policies configuration

    Returns:
        Success status and active policies
    """
    logger.info("Setting cooperation policies")
    return await ad_api.set_cooperation_policies(policies)


@mcp.tool()
async def send_cooperation_commands(
    commands: Dict[str, Any] = Field(..., description="Cooperation commands to send"),
) -> Dict[str, Any]:
    """
    Send cooperation commands.

    Args:
        commands: Cooperation commands

    Returns:
        Command acknowledgment
    """
    logger.info("Sending cooperation commands")
    return await ad_api.send_cooperation_commands(commands)


# System Monitoring
@mcp.tool()
async def monitor_diagnostics() -> Dict[str, Any]:
    """
    Get real-time diagnostics status.

    Returns:
        Comprehensive diagnostics information
    """
    logger.info("Getting diagnostics status")
    return await ad_api.get_diagnostics_status()


@mcp.tool()
async def reset_diagnostics() -> Dict[str, Any]:
    """
    Reset diagnostics.

    Returns:
        Success status and message
    """
    logger.info("Resetting diagnostics")
    return await ad_api.reset_diagnostics()


@mcp.tool()
async def monitor_system_heartbeat() -> Dict[str, Any]:
    """
    Monitor system heartbeat.

    Returns:
        System heartbeat status
    """
    logger.info("Getting system heartbeat")
    return await ad_api.get_system_heartbeat()


# Vehicle Information
@mcp.tool()
async def get_vehicle_state() -> Dict[str, Any]:
    """
    Get comprehensive vehicle state including dimensions, status, and kinematics.

    This unified tool combines vehicle dimensions, current status (speed, gear, steering),
    and kinematics (position, orientation, velocity) into a single response.

    Returns:
        Complete vehicle state information including:
        - dimensions: Vehicle physical dimensions
        - status: Current gear, speed, steering angle
        - kinematics: Position, orientation, velocities
        - timestamp: When the data was collected
    """
    logger.info("Getting comprehensive vehicle state")

    # Gather all vehicle information in parallel for efficiency
    dimensions = await ad_api.get_vehicle_dimensions()
    status = await ad_api.get_vehicle_status()
    kinematics = await ad_api.get_vehicle_kinematics()

    return {
        "success": all(
            [
                dimensions.get("success", False),
                status.get("success", False),
                kinematics.get("success", False),
            ]
        ),
        "dimensions": {
            "length": dimensions.get("length"),
            "width": dimensions.get("width"),
            "height": dimensions.get("height"),
            "wheelbase": dimensions.get("wheelbase"),
        },
        "status": {
            "gear": status.get("gear"),
            "speed": status.get("speed"),
            "steering_angle": status.get("steering_angle"),
        },
        "kinematics": {
            "position": kinematics.get("position"),
            "orientation": kinematics.get("orientation"),
            "velocity": kinematics.get("velocity"),
            "angular_velocity": kinematics.get("angular_velocity"),
        },
        "timestamp": datetime.now().isoformat(),
    }


# Internal server functions
async def _initialize_server() -> Dict[str, str]:
    """Internal function to initialize the MCP server."""
    logger.info("Initializing MCP server")

    try:
        # Initialize ROS2 manager
        ros2_initialized = await ros2_manager.initialize()

        # Get initial health status
        health = await monitor.get_complete_health_status()

        status = {
            "status": "initialized",
            "ros2_initialized": ros2_initialized,
            "overall_health": health["overall_status"],
            "message": "Server initialized successfully",
        }

        logger.info("Server initialization complete")
        return status

    except Exception as e:
        logger.error(f"Server initialization failed: {e}")
        return {"status": "failed", "message": str(e)}


async def _shutdown_server() -> Dict[str, str]:
    """Internal function to gracefully shutdown the MCP server."""
    logger.info("Shutting down MCP server")

    try:
        # Cleanup AD API resources
        await cleanup_ad_api()

        # Cleanup ROS2 resources
        await ros2_manager.cleanup()

        logger.info("Server shutdown complete")
        return {"status": "shutdown", "message": "Server shut down successfully"}

    except Exception as e:
        logger.error(f"Error during shutdown: {e}")
        return {"status": "error", "message": str(e)}


# Main entry point
async def main():
    """Main server entry point."""
    logger.info("Starting Autoware MCP server")

    # Initialize server
    await _initialize_server()

    try:
        # Start FastMCP server
        await mcp.run_async()
    finally:
        # Cleanup on shutdown
        await _shutdown_server()


if __name__ == "__main__":
    asyncio.run(main())
