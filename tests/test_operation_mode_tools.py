"""Test suite for operation mode MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .test_helpers import set_operation_mode, monitor_operation_mode
from autoware_mcp.server import OperationModeRequest
from autoware_mcp.ad_api_ros2 import OperationMode


@pytest.mark.asyncio
async def test_set_operation_mode_to_autonomous():
    """Test setting operation mode to autonomous."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import OperationModeResponse

        mock_api.change_to_autonomous = AsyncMock(
            return_value=OperationModeResponse(
                success=True,
                current_mode="autonomous",
                requested_mode="autonomous",
                message="Successfully changed to autonomous mode",
                timestamp="2025-01-19T12:00:00",
            )
        )

        request = OperationModeRequest(mode=OperationMode.AUTONOMOUS, transition_time=5)
        result = await set_operation_mode(request)

        assert result.success is True
        assert result.current_mode == "autonomous"
        mock_api.change_to_autonomous.assert_called_once()


@pytest.mark.asyncio
async def test_set_operation_mode_to_stop():
    """Test setting operation mode to stop."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import OperationModeResponse

        mock_api.change_to_stop = AsyncMock(
            return_value=OperationModeResponse(
                success=True,
                current_mode="stop",
                requested_mode="stop",
                message="Vehicle stopped",
                timestamp="2025-01-19T12:00:00",
            )
        )

        request = OperationModeRequest(mode=OperationMode.STOP)
        result = await set_operation_mode(request)

        assert result.success is True
        assert result.current_mode == "stop"


@pytest.mark.asyncio
async def test_monitor_operation_mode():
    """Test monitoring current operation mode."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_operation_mode_state = AsyncMock(
            return_value={
                "current_mode": "autonomous",
                "control_enabled": True,
                "ready_for_engage": True,
                "transition_state": "completed",
            }
        )

        result = await monitor_operation_mode()
        assert result["current_mode"] == "autonomous"
        assert result["control_enabled"] is True
        assert result["ready_for_engage"] is True
