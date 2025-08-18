"""Autoware Model Context Protocol (MCP) Server.

A FastMCP-based server for interacting with the Autoware autonomous driving stack
through ROS2 interfaces.
"""

__version__ = "0.1.0"

from .config import MCPConfig, get_config, set_config
from .ros2_manager import ROS2Manager, get_ros2_manager
from .health import SystemMonitor, get_monitor, HealthStatus
from .logging import setup_logging, get_logger
from .server import mcp, main
from .cli import run_tests

__all__ = [
    "MCPConfig",
    "get_config",
    "set_config",
    "ROS2Manager",
    "get_ros2_manager",
    "SystemMonitor",
    "get_monitor",
    "HealthStatus",
    "setup_logging",
    "get_logger",
    "mcp",
    "main",
    "run_tests",
]
