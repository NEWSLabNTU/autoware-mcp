"""Tests for perception tools."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from autoware_mcp.tools.perception_tools import PerceptionTools
from autoware_mcp.perception_bridge import PerceptionBridge


@pytest.fixture
def perception_tools():
    """Create perception tools instance for testing."""
    return PerceptionTools()


@pytest.mark.asyncio
async def test_capture_camera_view(perception_tools):
    """Test camera capture tool."""
    with patch.object(perception_tools.perception_bridge, 'capture_camera_image') as mock_capture:
        mock_capture.return_value = {
            "success": True,
            "image_path": "/tmp/autoware_mcp/perception/camera_12345.jpg",
            "timestamp": "2025-08-28T10:00:00",
            "width": 1920,
            "height": 1080
        }
        
        result = await perception_tools.capture_camera_view("front")
        
        assert result["success"] is True
        assert "image_path" in result
        assert result["camera"] == "front"
        assert "Use Read tool to view" in result["message"]
        mock_capture.assert_called_once()


@pytest.mark.asyncio
async def test_visualize_lidar_scene(perception_tools):
    """Test LiDAR visualization tool."""
    with patch.object(perception_tools.perception_bridge, 'visualize_pointcloud') as mock_viz:
        mock_viz.return_value = {
            "success": True,
            "image_path": "/tmp/autoware_mcp/perception/pointcloud_bev_12345.png",
            "timestamp": "2025-08-28T10:00:00",
            "num_points": 50000
        }
        
        result = await perception_tools.visualize_lidar_scene("bev")
        
        assert result["success"] is True
        assert "image_path" in result
        assert result["view_type"] == "bev"
        assert result["num_points"] == 50000
        mock_viz.assert_called_once()


@pytest.mark.asyncio
async def test_get_perception_snapshot(perception_tools):
    """Test perception snapshot tool."""
    with patch.object(perception_tools.perception_bridge, 'get_perception_snapshot') as mock_snap:
        mock_snap.return_value = {
            "success": True,
            "timestamp": "2025-08-28T10:00:00",
            "images": {
                "camera": "/tmp/autoware_mcp/perception/camera.jpg",
                "lidar_bev": "/tmp/autoware_mcp/perception/lidar.png",
                "annotated": "/tmp/autoware_mcp/perception/annotated.jpg"
            },
            "detected_objects": [
                {"type": "vehicle", "id": 1, "position": {"x": 10, "y": 5}}
            ]
        }
        
        result = await perception_tools.get_perception_snapshot()
        
        assert result["success"] is True
        assert "camera_image" in result
        assert "lidar_visualization" in result
        assert "annotated_view" in result
        assert result["num_objects"] == 1
        mock_snap.assert_called_once()


@pytest.mark.asyncio
async def test_analyze_driving_scene(perception_tools):
    """Test driving scene analysis tool."""
    with patch.object(perception_tools.perception_bridge, 'analyze_driving_scene') as mock_analyze:
        mock_analyze.return_value = {
            "success": True,
            "timestamp": "2025-08-28T10:00:00",
            "images": {"camera": "path1", "lidar_bev": "path2", "annotated": "path3"},
            "scene_description": {"num_vehicles": 3, "num_pedestrians": 2},
            "detected_objects": [],
            "traffic_lights": [{"id": 1, "color": "green"}],
            "recommendations": ["Watch for pedestrians"]
        }
        
        result = await perception_tools.analyze_driving_scene()
        
        assert result["success"] is True
        assert "images" in result
        assert "scene_summary" in result
        assert result["scene_summary"]["num_vehicles"] == 3
        assert len(result["recommendations"]) == 1
        mock_analyze.assert_called_once()


@pytest.mark.asyncio
async def test_get_detected_objects(perception_tools):
    """Test object detection tool."""
    with patch.object(perception_tools.perception_bridge, '_get_detected_objects') as mock_detect:
        mock_detect.return_value = [
            {"type": "vehicle", "id": 1, "position": {"x": 10, "y": 5}},
            {"type": "pedestrian", "id": 2, "position": {"x": 20, "y": 10}},
            {"type": "vehicle", "id": 3, "position": {"x": 30, "y": 15}}
        ]
        
        # Test all objects
        result = await perception_tools.get_detected_objects("all")
        assert result["success"] is True
        assert result["count"] == 3
        assert result["object_type"] == "all"
        
        # Test filtered objects
        result = await perception_tools.get_detected_objects("vehicle")
        assert result["success"] is True
        assert result["count"] == 2
        assert result["object_type"] == "vehicle"
        assert all(obj["type"] == "vehicle" for obj in result["objects"])


@pytest.mark.asyncio
async def test_error_handling(perception_tools):
    """Test error handling in perception tools."""
    with patch.object(perception_tools.perception_bridge, 'capture_camera_image') as mock_capture:
        mock_capture.return_value = {
            "success": False,
            "error": "Camera not available"
        }
        
        result = await perception_tools.capture_camera_view("front")
        
        assert result["success"] is False
        assert "error" in result
        assert result["camera"] == "front"


def test_perception_tools_initialization():
    """Test that perception tools can be initialized."""
    tools = PerceptionTools()
    assert tools.perception_bridge is not None
    
    # Check that tool methods exist
    assert hasattr(tools, 'capture_camera_view')
    assert hasattr(tools, 'visualize_lidar_scene')
    assert hasattr(tools, 'get_perception_snapshot')
    assert hasattr(tools, 'analyze_driving_scene')
    assert hasattr(tools, 'get_detected_objects')


def test_perception_bridge_initialization():
    """Test that perception bridge can be initialized."""
    bridge = PerceptionBridge()
    assert bridge.media_dir.exists()
    assert bridge.max_files == 100
    assert bridge.generated_files == []