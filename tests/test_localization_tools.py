"""Test suite for localization MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .helpers import initialize_localization, monitor_localization_state
from autoware_mcp.server import LocalizationRequest


@pytest.mark.asyncio
async def test_initialize_localization():
    """Test initializing localization with a pose."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import LocalizationResponse

        mock_api.initialize_localization = AsyncMock(
            return_value=LocalizationResponse(
                success=True,
                message="Localization initialized successfully",
                initialization_state="initialized",
            )
        )

        request = LocalizationRequest(
            pose={
                "position": {"x": 100.0, "y": 200.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707},
            }
        )
        result = await initialize_localization(request)

        assert result.success is True
        assert "initialized successfully" in result.message


@pytest.mark.asyncio
async def test_initialize_localization_with_covariance():
    """Test initializing localization with pose and covariance."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import LocalizationResponse

        mock_api.initialize_localization = AsyncMock(
            return_value=LocalizationResponse(
                success=True,
                message="Localization initialized with covariance",
                initialization_state="initialized_with_covariance",
            )
        )

        request = LocalizationRequest(
            pose={
                "position": {"x": 100.0, "y": 200.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            pose_with_covariance=[0.1] * 36,  # 6x6 covariance matrix
        )
        result = await initialize_localization(request)

        assert result.success is True
        assert "covariance" in result.message


@pytest.mark.asyncio
async def test_monitor_localization_state():
    """Test monitoring localization state."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_localization_state = AsyncMock(
            return_value={
                "initialized": True,
                "quality": "good",
                "confidence": 0.95,
                "position_error": 0.05,
                "orientation_error": 0.02,
                "current_pose": {
                    "position": {"x": 100.5, "y": 200.3, "z": 0.1},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.01, "w": 0.999},
                },
            }
        )

        result = await monitor_localization_state()
        assert result["initialized"] is True
        assert result["quality"] == "good"
        assert result["confidence"] == 0.95
        assert result["position_error"] == 0.05
