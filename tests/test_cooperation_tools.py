"""Test suite for cooperation MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .test_helpers import (
    get_cooperation_policies,
    set_cooperation_policies,
    send_cooperation_commands,
)


@pytest.mark.asyncio
async def test_get_cooperation_policies():
    """Test getting current cooperation policies."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_cooperation_policies = AsyncMock(
            return_value={
                "policies": {
                    "lane_change": "cooperative",
                    "intersection": "cautious",
                    "merge": "assertive",
                },
                "active": True,
            }
        )

        result = await get_cooperation_policies()
        assert result["active"] is True
        assert result["policies"]["lane_change"] == "cooperative"
        assert result["policies"]["intersection"] == "cautious"


@pytest.mark.asyncio
async def test_set_cooperation_policies():
    """Test setting cooperation policies."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.set_cooperation_policies = AsyncMock(
            return_value={
                "success": True,
                "policies": {"lane_change": "assertive", "intersection": "cooperative"},
                "message": "Policies updated successfully",
            }
        )

        policies = {"lane_change": "assertive", "intersection": "cooperative"}
        result = await set_cooperation_policies(policies=policies)

        assert result["success"] is True
        assert result["policies"]["lane_change"] == "assertive"


@pytest.mark.asyncio
async def test_send_cooperation_commands():
    """Test sending cooperation commands."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.send_cooperation_commands = AsyncMock(
            return_value={
                "success": True,
                "commands_sent": ["request_lane_change", "yield_to_pedestrian"],
                "message": "Cooperation commands sent",
            }
        )

        commands = {
            "action": "request_lane_change",
            "target_lane": "left",
            "urgency": "normal",
        }
        result = await send_cooperation_commands(commands=commands)

        assert result["success"] is True
        assert "request_lane_change" in result["commands_sent"]
