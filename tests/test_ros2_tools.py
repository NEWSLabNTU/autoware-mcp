"""Test suite for ROS2-specific MCP tools."""

import pytest
from unittest.mock import AsyncMock, patch

from .helpers import (
    list_ros2_nodes,
    list_ros2_topics,
    list_ros2_services,
    get_node_info,
    get_topic_info,
    get_topic_frequency,
    echo_topic_messages,
    call_ros2_service,
    publish_to_topic,
)


@pytest.mark.asyncio
async def test_list_ros2_nodes():
    """Test listing ROS2 nodes."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.list_nodes = AsyncMock(
            return_value=[
                "/perception/lidar_centerpoint",
                "/planning/behavior_planner",
                "/control/trajectory_follower",
            ]
        )

        result = await list_ros2_nodes()
        assert len(result) == 3
        assert "/perception/lidar_centerpoint" in result
        assert "/control/trajectory_follower" in result


@pytest.mark.asyncio
async def test_list_ros2_topics():
    """Test listing ROS2 topics."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.list_topics = AsyncMock(
            return_value=[
                "/tf",
                "/tf_static",
                "/sensing/lidar/concatenated/pointcloud",
                "/planning/scenario_planning/trajectory",
            ]
        )

        result = await list_ros2_topics()
        assert len(result) == 4
        assert "/tf" in result
        assert "/planning/scenario_planning/trajectory" in result


@pytest.mark.asyncio
async def test_list_ros2_services():
    """Test listing ROS2 services."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.list_services = AsyncMock(
            return_value=[
                "/api/routing/set_route",
                "/api/routing/clear_route",
                "/api/operation_mode/change_to_autonomous",
            ]
        )

        result = await list_ros2_services()
        assert len(result) == 3
        assert "/api/routing/set_route" in result


@pytest.mark.asyncio
async def test_get_node_info():
    """Test getting node information."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.get_node_info = AsyncMock(
            return_value={
                "name": "/perception/lidar_centerpoint",
                "namespace": "/perception",
                "publishers": ["/perception/objects"],
                "subscribers": ["/sensing/lidar/concatenated/pointcloud"],
                "services": [],
                "parameters": {"score_threshold": 0.4},
            }
        )

        result = await get_node_info(node_name="/perception/lidar_centerpoint")
        assert result["name"] == "/perception/lidar_centerpoint"
        assert "/perception/objects" in result["publishers"]
        assert result["parameters"]["score_threshold"] == 0.4


@pytest.mark.asyncio
async def test_get_topic_info():
    """Test getting topic information."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.get_topic_info = AsyncMock(
            return_value={
                "name": "/tf",
                "message_type": "tf2_msgs/msg/TFMessage",
                "publishers": 5,
                "subscribers": 12,
            }
        )

        result = await get_topic_info(topic_name="/tf")
        assert result["name"] == "/tf"
        assert result["message_type"] == "tf2_msgs/msg/TFMessage"
        assert result["publishers"] == 5


@pytest.mark.asyncio
async def test_get_topic_frequency():
    """Test measuring topic frequency."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.get_topic_hz = AsyncMock(
            return_value={
                "topic": "/sensing/lidar/concatenated/pointcloud",
                "frequency_hz": 10.2,
                "message_count": 51,
                "duration_seconds": 5.0,
            }
        )

        result = await get_topic_frequency(
            topic_name="/sensing/lidar/concatenated/pointcloud", duration=5
        )
        assert result["frequency_hz"] == 10.2
        assert result["message_count"] == 51


@pytest.mark.asyncio
async def test_echo_topic_messages():
    """Test echoing topic messages."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.initialize = AsyncMock()
        mock_ros2.echo_topic = AsyncMock(
            return_value={
                "topic": "/tf",
                "messages": [
                    {"transforms": [{"header": {"stamp": {"sec": 1234567890}}}]},
                    {"transforms": [{"header": {"stamp": {"sec": 1234567891}}}]},
                ],
                "count": 2,
            }
        )

        result = await echo_topic_messages(topic_name="/tf", count=2)
        assert result["count"] == 2
        assert len(result["messages"]) == 2
        assert (
            result["messages"][0]["transforms"][0]["header"]["stamp"]["sec"]
            == 1234567890
        )


@pytest.mark.asyncio
async def test_call_ros2_service():
    """Test calling a ROS2 service."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.call_service = AsyncMock(
            return_value={
                "success": True,
                "data": {"success": True, "message": "Route cleared"},
            }
        )

        result = await call_ros2_service(
            service_name="/api/routing/clear_route",
            service_type="autoware_adapi_v1_msgs/srv/ClearRoute",
            request={},
        )
        assert result["success"] is True
        assert result["response"]["success"] is True


@pytest.mark.asyncio
async def test_publish_to_topic():
    """Test publishing to a ROS2 topic."""
    with patch("autoware_mcp.server.ros2_manager") as mock_ros2:
        mock_ros2.publish_to_topic = AsyncMock(return_value=True)

        result = await publish_to_topic(
            topic_name="/initialpose",
            message_type="geometry_msgs/msg/PoseWithCovarianceStamped",
            message={
                "header": {"frame_id": "map"},
                "pose": {
                    "pose": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    }
                },
            },
        )
        assert result["success"] is True
        assert result["error"] is None
