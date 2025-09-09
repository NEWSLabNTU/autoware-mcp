"""Test suite for system-level MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .helpers import (
    health_check,
    get_system_status,
    verify_ros2_environment,
    get_configuration,
    monitor_system_heartbeat,
    monitor_diagnostics,
    reset_diagnostics,
)


@pytest.mark.asyncio
async def test_health_check():
    """Test health_check tool returns proper health status."""
    with patch("autoware_mcp.server.monitor") as mock_monitor:
        mock_monitor.get_complete_health_status = AsyncMock(
            return_value={
                "overall_status": "healthy",
                "timestamp": "2025-01-19T12:00:00",
                "uptime_seconds": 3600.0,
                "system": {
                    "cpu_percent": 25.0,
                    "memory_percent": 40.0,
                    "disk_percent": 60.0,
                },
                "ros2": {"available": True, "node_count": 15, "topic_count": 25},
                "autoware": {
                    "workspace": "/home/user/autoware",
                    "components_active": 10,
                },
            }
        )

        result = await health_check()
        # Result is a HealthResponse model, access attributes directly
        assert result.status == "healthy"
        assert result.uptime_seconds == 3600.0
        assert result.system["cpu_percent"] == 25.0
        assert result.ros2["available"] is True
        assert result.autoware["components_active"] == 10


@pytest.mark.asyncio
async def test_get_system_status():
    """Test get_system_status returns system resource information."""
    with patch("autoware_mcp.server.monitor") as mock_monitor:
        mock_monitor.get_system_info = AsyncMock(
            return_value={
                "cpu": {"percent": 30.0, "frequency": 2400.0},
                "memory": {"percent": 45.0, "available_gb": 8.5},
                "disk": {"percent": 55.0, "free_gb": 100.0},
                "network": {"interfaces": ["eth0", "lo"]},
                "processes": {"total": 250, "running": 2},
            }
        )

        result = await get_system_status()
        assert result["cpu"]["percent"] == 30.0
        assert result["memory"]["available_gb"] == 8.5
        assert "eth0" in result["network"]["interfaces"]
        assert result["processes"]["total"] == 250


@pytest.mark.asyncio
async def test_verify_ros2_environment():
    """Test verify_ros2_environment checks ROS2 setup."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.verify_environment = AsyncMock(
            return_value={
                "ros2_available": True,
                "ros_distro": "humble",
                "ros_version": "2.0.0",
                "ready": True,
            }
        )

        result = await verify_ros2_environment()
        # The verify_ros2_environment function returns its own structure
        assert "valid" in result
        assert "checks" in result
        assert isinstance(result["valid"], bool)


@pytest.mark.asyncio
async def test_get_configuration():
    """Test get_configuration returns server configuration."""
    with patch("autoware_mcp.server.config") as mock_config:
        mock_config.dict.return_value = {
            "server": {"port": 8080, "log_level": "INFO"},
            "autoware": {"workspace_path": "/opt/autoware", "ros_distro": "humble"},
        }

        result = await get_configuration()
        assert result["server"]["port"] == 8080
        assert result["autoware"]["ros_distro"] == "humble"


@pytest.mark.asyncio
async def test_monitor_system_heartbeat():
    """Test monitor_system_heartbeat returns heartbeat status."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_system_heartbeat = AsyncMock(
            return_value={
                "alive": True,
                "last_heartbeat": "2025-01-19T12:00:00",
                "uptime_seconds": 7200.0,
                "checks_passed": 100,
                "checks_failed": 0,
            }
        )

        result = await monitor_system_heartbeat()
        assert result["alive"] is True
        assert result["uptime_seconds"] == 7200.0
        assert result["checks_failed"] == 0


@pytest.mark.asyncio
async def test_monitor_diagnostics():
    """Test monitor_diagnostics returns diagnostic information."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_diagnostics_status = AsyncMock(
            return_value={
                "status": "OK",
                "errors": [],
                "warnings": ["Low memory warning"],
                "component_status": {
                    "perception": "OK",
                    "planning": "OK",
                    "control": "OK",
                },
            }
        )

        result = await monitor_diagnostics()
        assert result["status"] == "OK"
        assert len(result["warnings"]) == 1
        assert result["component_status"]["perception"] == "OK"


@pytest.mark.asyncio
async def test_reset_diagnostics():
    """Test reset_diagnostics clears diagnostic state."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.reset_diagnostics = AsyncMock(
            return_value={"success": True, "message": "Diagnostics reset successfully"}
        )

        result = await reset_diagnostics()
        assert result["success"] is True
        assert "reset successfully" in result["message"]
