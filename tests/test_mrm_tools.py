"""Test suite for Minimum Risk Maneuver (MRM) MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .test_helpers import request_mrm, list_mrm_behaviors, monitor_mrm_state
from autoware_mcp.server import MRMRequest


@pytest.mark.asyncio
async def test_request_mrm():
    """Test requesting a Minimum Risk Maneuver."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import MRMResponse

        mock_api.request_mrm = AsyncMock(
            return_value=MRMResponse(
                success=True,
                message="Emergency stop initiated",
                current_state="executing",
            )
        )

        request = MRMRequest(behavior="emergency_stop", reason="Obstacle detected")
        result = await request_mrm(request)

        assert result.success is True
        # Check that result has expected fields
        assert result.current_state == "executing"


@pytest.mark.asyncio
async def test_request_mrm_pull_over():
    """Test requesting a pull over MRM."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import MRMResponse

        mock_api.request_mrm = AsyncMock(
            return_value=MRMResponse(
                success=True,
                message="Searching for safe pull over location",
                current_state="searching_safe_spot",
            )
        )

        request = MRMRequest(behavior="pull_over", reason="System failure detected")
        result = await request_mrm(request)

        assert result.success is True
        # Check that result has expected state
        assert "safe" in result.message.lower()


@pytest.mark.asyncio
async def test_list_mrm_behaviors():
    """Test listing available MRM behaviors."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.list_mrm_behaviors = AsyncMock(
            return_value=[
                "emergency_stop",
                "pull_over",
                "comfortable_stop",
                "slow_down",
            ]
        )

        result = await list_mrm_behaviors()
        assert len(result) == 4
        assert "emergency_stop" in result
        assert "pull_over" in result
        assert "comfortable_stop" in result


@pytest.mark.asyncio
async def test_monitor_mrm_state():
    """Test monitoring MRM state."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_mrm_state = AsyncMock(
            return_value={
                "mrm_active": False,
                "current_behavior": None,
                "available_behaviors": ["emergency_stop", "pull_over"],
                "last_mrm_time": "2025-01-19T11:30:00",
                "ready": True,
            }
        )

        result = await monitor_mrm_state()
        assert result["mrm_active"] is False
        assert result["current_behavior"] is None
        assert "emergency_stop" in result["available_behaviors"]
        assert result["ready"] is True
