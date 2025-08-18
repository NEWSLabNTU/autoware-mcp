"""Main MCP server implementation using FastMCP."""

import asyncio
import os
from typing import Dict, Any, List, Optional
from pathlib import Path

from fastmcp import FastMCP
from pydantic import BaseModel, Field

from .config import get_config, MCPConfig
from .ros2_manager import get_ros2_manager
from .health import get_monitor, HealthStatus
from .logging import setup_logging, get_logger

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
    topic_name: str = Field(description="Name of the ROS2 topic")
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
    duration: float = Field(default=5.0, description="Duration to measure frequency (seconds)")
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
    count: int = Field(default=1, description="Number of messages to capture")
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
            ["ros2", "topic", "list", "--spin-time", "0"],
            timeout=5.0
        )
        result["checks"]["can_list_topics"] = topics_result["returncode"] == 0
    except Exception:
        result["checks"]["can_list_topics"] = False

    # Determine overall validity
    result["valid"] = (
        result["checks"].get("ros2_available", False) and
        len(result["errors"]) == 0
    )

    logger.info(
        f"Environment verification result: {'valid' if result['valid'] else 'invalid'}"
    )
    return result


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
