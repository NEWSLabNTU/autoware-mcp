"""Test suite for routing MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .helpers import set_route, set_route_points, get_current_route
from autoware_mcp.server import RouteRequest


@pytest.mark.asyncio
async def test_set_route():
    """Test setting a route to a goal pose."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import RouteResponse

        mock_api.set_route = AsyncMock(
            return_value=RouteResponse(
                success=True,
                message="Route set successfully",
                route_id="route_123",
                distance=500.0,
                duration=60.0,
            )
        )

        request = RouteRequest(
            goal_pose={
                "position": {"x": 1000.0, "y": 2000.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        )
        result = await set_route(request)

        assert result.success is True
        assert result.route_id == "route_123"
        assert result.distance == 500.0


@pytest.mark.asyncio
async def test_set_route_with_waypoints():
    """Test setting a route with waypoints."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import RouteResponse

        mock_api.set_route_points = AsyncMock(
            return_value=RouteResponse(
                success=True,
                message="Route with waypoints set",
                route_id="route_456",
                distance=750.0,
            )
        )

        request = RouteRequest(
            goal_pose={
                "position": {"x": 1000.0, "y": 2000.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            waypoints=[
                {"position": {"x": 250.0, "y": 500.0, "z": 0.0}},
                {"position": {"x": 500.0, "y": 1000.0, "z": 0.0}},
                {"position": {"x": 750.0, "y": 1500.0, "z": 0.0}},
            ],
        )
        result = await set_route(request)

        assert result.success is True
        assert result.route_id == "route_456"


@pytest.mark.asyncio
async def test_set_route_points():
    """Test setting a route with specific waypoints."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        from autoware_mcp.ad_api_ros2 import RouteResponse

        mock_api.set_route_points = AsyncMock(
            return_value=RouteResponse(
                success=True,
                message="Route points set",
                route_id="route_789",
                distance=1000.0,
            )
        )

        waypoints = [
            {"position": {"x": 0.0, "y": 0.0, "z": 0.0}},
            {"position": {"x": 500.0, "y": 500.0, "z": 0.0}},
            {"position": {"x": 1000.0, "y": 1000.0, "z": 0.0}},
        ]

        result = await set_route_points(waypoints=waypoints)
        assert result.success is True
        assert result.route_id == "route_789"


@pytest.mark.asyncio
async def test_get_current_route():
    """Test getting current route information."""
    with patch("autoware_mcp.server.ad_api") as mock_api:
        mock_api.get_route_state = AsyncMock(
            return_value={
                "has_route": True,
                "route_id": "route_current",
                "progress_percentage": 45.0,
                "remaining_distance_m": 275.0,
                "waypoints_passed": 2,
                "waypoints_total": 5,
            }
        )

        result = await get_current_route()
        assert result["has_route"] is True
        assert result["progress_percentage"] == 45.0
        assert result["waypoints_passed"] == 2
