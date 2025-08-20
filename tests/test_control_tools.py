"""Test suite for vehicle control MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .test_helpers import (
    send_velocity_command,
    send_acceleration_command,
    send_steering_command,
    send_pedals_command,
    monitor_motion_state,
)


@pytest.mark.asyncio
async def test_send_velocity_command():
    """Test sending velocity control command."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.send_velocity_command = AsyncMock(
            return_value={
                "success": True,
                "command": "velocity",
                "value": 10.0,
                "unit": "m/s",
            }
        )

        result = await send_velocity_command(velocity=10.0)
        assert result["success"] is True
        assert result["value"] == 10.0
        assert result["unit"] == "m/s"


@pytest.mark.asyncio
async def test_send_acceleration_command():
    """Test sending acceleration control command."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.send_acceleration_command = AsyncMock(
            return_value={
                "success": True,
                "command": "acceleration",
                "value": 2.0,
                "unit": "m/s²",
            }
        )

        result = await send_acceleration_command(acceleration=2.0)
        assert result["success"] is True
        assert result["value"] == 2.0


@pytest.mark.asyncio
async def test_send_steering_command():
    """Test sending steering control command."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.send_steering_command = AsyncMock(
            return_value={
                "success": True,
                "command": "steering",
                "value": 0.3,
                "unit": "radians",
            }
        )

        result = await send_steering_command(steering_angle=0.3)
        assert result["success"] is True
        assert result["value"] == 0.3
        assert result["unit"] == "radians"


@pytest.mark.asyncio
async def test_send_pedals_command():
    """Test sending pedals control command."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.send_pedals_command = AsyncMock(
            return_value={
                "success": True,
                "throttle": 0.3,
                "brake": 0.0,
                "message": "Pedals command sent",
            }
        )

        result = await send_pedals_command(throttle=0.3, brake=0.0)
        assert result["success"] is True
        assert result["throttle"] == 0.3
        assert result["brake"] == 0.0


@pytest.mark.asyncio
async def test_send_pedals_brake():
    """Test sending brake pedal command."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.send_pedals_command = AsyncMock(
            return_value={
                "success": True,
                "throttle": 0.0,
                "brake": 0.7,
                "message": "Braking applied",
            }
        )

        result = await send_pedals_command(throttle=0.0, brake=0.7)
        assert result["success"] is True
        assert result["brake"] == 0.7
        assert result["throttle"] == 0.0


@pytest.mark.asyncio
async def test_monitor_motion_state():
    """Test monitoring motion state."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_motion_state = AsyncMock(
            return_value={
                "motion_ready": True,
                "velocity_mps": 5.0,
                "acceleration_mps2": 0.5,
                "steering_angle_rad": 0.1,
                "trajectory_following": True,
                "control_mode": "trajectory",
            }
        )

        result = await monitor_motion_state()
        assert result["motion_ready"] is True
        assert result["velocity_mps"] == 5.0
        assert result["trajectory_following"] is True
