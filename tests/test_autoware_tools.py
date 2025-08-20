"""Test suite for Autoware-specific MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .test_helpers import check_autoware_status, get_vehicle_state


@pytest.mark.asyncio
async def test_check_autoware_status():
    """Test checking Autoware component status."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.check_autoware_nodes = AsyncMock(
            return_value={
                "autoware_running": True,
                "autoware_detected": True,
                "active_components": {
                    "perception": ["lidar_centerpoint", "traffic_light_classifier"],
                    "planning": ["behavior_planner", "motion_planner"],
                    "control": ["trajectory_follower", "vehicle_cmd_gate"],
                },
                "component_count": 6,
                "status": "operational",
            }
        )

        result = await check_autoware_status()
        # Check if result is a dict or has dict-like properties
        assert "autoware_running" in result
        assert result["autoware_running"] is True
        assert result["component_count"] == 6
        assert "lidar_centerpoint" in result["active_components"]["perception"]
        assert result["status"] == "operational"


@pytest.mark.asyncio
async def test_get_vehicle_state():
    """Test getting comprehensive vehicle state."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_vehicle_dimensions = AsyncMock(
            return_value={
                "success": True,
                "length": 4.5,
                "width": 1.8,
                "height": 1.5,
                "wheelbase": 2.7,
            }
        )
        mock_api.get_vehicle_status = AsyncMock(
            return_value={
                "success": True,
                "gear": "DRIVE",
                "speed": 5.0,
                "steering_angle": 0.1,
            }
        )
        mock_api.get_vehicle_kinematics = AsyncMock(
            return_value={
                "success": True,
                "position": {"x": 100.0, "y": 200.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 1.57},
                "velocity": {"linear": 5.0, "angular": 0.1},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
            }
        )

        result = await get_vehicle_state()
        assert result["dimensions"]["length"] == 4.5
        assert result["status"]["speed"] == 5.0
        assert result["kinematics"]["position"]["x"] == 100.0
