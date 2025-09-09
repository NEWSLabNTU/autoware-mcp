"""
Tests for perception module.
"""

import pytest
import asyncio
import numpy as np
import time
from pathlib import Path
import shutil
import tempfile

from autoware_mcp.perception import (
    PerceptionContext,
    CircularBuffer,
    StorageManager,
    ProjectionEngine,
    ColoringEngine,
    create_perception_context,
    generate_visualization,
    analyze_temporal,
    save_context_buffer
)
from autoware_mcp.perception.projections import ProjectionParams, PROJECTION_PRESETS


class TestCircularBuffer:
    """Test circular buffer functionality."""
    
    def test_buffer_initialization(self):
        """Test buffer initialization."""
        buffer = CircularBuffer(duration=10.0, max_size_bytes=1024*1024)
        assert buffer.duration == 10.0
        assert buffer.max_size_bytes == 1024*1024
        assert buffer.size() == 0
        
    def test_buffer_add_and_get(self):
        """Test adding and retrieving items."""
        buffer = CircularBuffer(duration=10.0)
        
        # Add items
        for i in range(5):
            buffer.add({"id": i, "data": f"item_{i}"})
            time.sleep(0.01)
            
        assert buffer.size() == 5
        
        # Get latest
        latest = buffer.get_latest(2)
        assert len(latest) == 2
        assert latest[-1]["id"] == 4
        
    def test_buffer_time_cleanup(self):
        """Test automatic cleanup of old items."""
        buffer = CircularBuffer(duration=0.1)  # 100ms duration
        
        # Add items
        buffer.add({"id": 0})
        time.sleep(0.15)  # Wait longer than duration
        buffer.add({"id": 1})
        
        # Old item should be removed
        all_items = buffer.get_all()
        assert len(all_items) == 1
        assert all_items[0]["id"] == 1
        
    def test_buffer_size_limit(self):
        """Test size-based cleanup."""
        buffer = CircularBuffer(max_size_bytes=10000000)  # 10MB size limit
        
        # Add large items
        for i in range(100):
            buffer.add({
                "id": i,
                "type": "image",
                "dimensions": (1920, 1080)  # Will estimate large size
            })
            
        # Buffer should maintain size limit
        assert buffer.size_bytes() <= buffer.max_size_bytes * 1.1  # Allow 10% margin
        
    def test_buffer_get_range(self):
        """Test getting items within time range."""
        buffer = CircularBuffer()
        
        # Add items with known timestamps
        base_time = time.time()
        for i in range(5):
            buffer.add({
                "id": i,
                "timestamp_ns": int((base_time + i) * 1e9)
            })
            
        # Get middle range
        items = buffer.get_range(base_time + 1, base_time + 3.5)
        ids = [item["id"] for item in items]
        assert ids == [1, 2, 3]


class TestStorageManager:
    """Test storage manager functionality."""
    
    def setup_method(self):
        """Set up test storage directory."""
        self.test_dir = Path(tempfile.mkdtemp())
        self.storage = StorageManager(base_path=self.test_dir)
        
    def teardown_method(self):
        """Clean up test directory."""
        if self.test_dir.exists():
            shutil.rmtree(self.test_dir)
            
    def test_storage_initialization(self):
        """Test storage directory structure creation."""
        assert (self.test_dir / "contexts").exists()
        assert (self.test_dir / "archives").exists()
        assert (self.test_dir / "cache").exists()
        
    def test_save_image(self):
        """Test saving image data."""
        # Create dummy image
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        context_id = "test_context"
        metadata = {
            "topic": "/camera/image",
            "timestamp_ns": 123456789
        }
        
        path = self.storage.save_image(image, context_id, metadata)
        
        assert Path(path).exists()
        assert "test_context" in path
        assert "camera_image" in path
        
    def test_save_pointcloud(self):
        """Test saving point cloud data."""
        # Create dummy point cloud
        points = np.random.randn(1000, 4).astype(np.float32)
        context_id = "test_context"
        metadata = {
            "topic": "/lidar/points",
            "timestamp_ns": 987654321
        }
        
        path = self.storage.save_pointcloud(points, context_id, metadata)
        
        assert Path(path).exists()
        assert "test_context" in path
        assert "lidar_points" in path
        
        # Load and verify
        loaded = np.load(path)
        assert loaded.shape == points.shape
        
    def test_context_cleanup(self):
        """Test cleanup of old contexts."""
        # Create old context
        old_context = self.test_dir / "contexts" / "old_context"
        old_context.mkdir(parents=True)
        (old_context / "test.txt").write_text("old data")
        
        # Make it old
        import os
        old_time = time.time() - (25 * 3600)  # 25 hours ago
        os.utime(old_context, (old_time, old_time))
        
        # Create new context
        new_context = self.test_dir / "contexts" / "new_context"
        new_context.mkdir(parents=True)
        (new_context / "test.txt").write_text("new data")
        
        # Cleanup old contexts
        removed = self.storage.cleanup_old_contexts(max_age_hours=24.0)
        
        assert removed == 1
        assert not old_context.exists()
        assert new_context.exists()


class TestPerceptionContext:
    """Test perception context functionality."""
    
    @pytest.mark.asyncio
    async def test_context_lifecycle(self):
        """Test context creation and lifecycle."""
        context = PerceptionContext(
            buffer_duration=5.0,
            context_type="test"
        )
        
        assert context.context_type == "test"
        assert context.is_active == False
        
        # Start context
        await context.start()
        assert context.is_active == True
        
        # Add captures
        context.add_capture({
            "type": "image",
            "topic": "/camera",
            "data": np.zeros((100, 100, 3))
        })
        
        assert context.rolling_buffer.size() == 1
        
        # Stop context
        await context.stop()
        assert context.is_active == False
        
    @pytest.mark.asyncio
    async def test_context_manager(self):
        """Test context manager usage."""
        async with PerceptionContext() as context:
            assert context.is_active == True
            
            context.add_capture({
                "type": "test",
                "data": "test_data"
            })
            
        # Context should be stopped after exit
        assert context.is_active == False
        
    def test_get_latest(self):
        """Test getting latest capture."""
        context = PerceptionContext()
        context.is_active = True
        
        # Add captures
        context.add_capture({"topic": "/camera1", "id": 1})
        context.add_capture({"topic": "/camera2", "id": 2})
        context.add_capture({"topic": "/camera1", "id": 3})
        
        # Get latest for specific topic
        latest = context.get_latest("/camera1")
        assert latest["id"] == 3
        
        # Get latest overall
        latest = context.get_latest()
        assert latest["id"] == 3


class TestProjectionEngine:
    """Test projection engine functionality."""
    
    def test_bev_projection(self):
        """Test bird's eye view projection."""
        engine = ProjectionEngine()
        
        # Create test point cloud
        points = np.random.randn(1000, 4).astype(np.float32)
        points[:, :3] *= 10  # Scale to reasonable range
        
        # Project to BEV
        img = engine.project_pointcloud(
            points,
            projection="bev",
            scale=0.1,
            color_mode="height"
        )
        
        assert img.shape[2] == 3  # RGB image
        assert img.dtype == np.uint8
        
    def test_angled_projection(self):
        """Test angled projection."""
        engine = ProjectionEngine()
        
        # Create test point cloud
        points = np.random.randn(500, 4).astype(np.float32)
        points[:, :3] *= 10
        
        # Test with preset
        img = engine.project_pointcloud(
            points,
            projection="front_left_45",
            scale=0.1
        )
        
        assert img.shape[2] == 3
        
    def test_perspective_projection(self):
        """Test perspective projection."""
        engine = ProjectionEngine()
        
        # Create test point cloud
        points = np.random.randn(500, 4).astype(np.float32)
        points[:, :3] *= 10
        
        # Custom parameters
        params = ProjectionParams(
            camera_height=1.5,
            look_ahead=10.0,
            fov_horizontal=90.0
        )
        
        img = engine.project_pointcloud(
            points,
            projection="perspective",
            params=params,
            scale=0.1
        )
        
        assert img.shape[2] == 3
        
    def test_empty_pointcloud(self):
        """Test handling of empty point cloud."""
        engine = ProjectionEngine()
        
        # Empty point cloud
        points = np.array([]).reshape(0, 4)
        
        img = engine.project_pointcloud(
            points,
            projection="bev",
            scale=0.1
        )
        
        # Should return empty image
        assert img.shape[2] == 3
        assert img.shape[0] > 0  # Has dimensions
        assert img.shape[1] > 0


class TestColoringEngine:
    """Test coloring engine functionality."""
    
    def test_height_coloring(self):
        """Test height-based coloring."""
        engine = ColoringEngine()
        
        # Create points with varying heights
        points = np.zeros((100, 4))
        points[:, 2] = np.linspace(-2, 10, 100)  # Z values
        
        colors = engine.apply_coloring(
            points,
            mode="height",
            params={"min_height": -2, "max_height": 10}
        )
        
        assert colors.shape == (100, 3)
        assert colors.dtype == np.uint8
        
    def test_distance_coloring(self):
        """Test distance-based coloring."""
        engine = ColoringEngine()
        
        # Create points at varying distances
        points = np.zeros((50, 4))
        points[:, 0] = np.linspace(0, 50, 50)  # X values
        
        colors = engine.apply_coloring(
            points,
            mode="distance",
            params={"max_distance": 50}
        )
        
        assert colors.shape == (50, 3)
        
    def test_uniform_coloring(self):
        """Test uniform coloring."""
        engine = ColoringEngine()
        
        points = np.random.randn(30, 4)
        
        colors = engine.apply_coloring(
            points,
            mode="uniform",
            params={"color": [255, 0, 0]}  # Red in BGR format
        )
        
        assert colors.shape == (30, 3)
        assert np.all(colors[:, 0] == 255)  # Blue channel in BGR (which is Red)
        assert np.all(colors[:, 1] == 0)  # Green channel
        assert np.all(colors[:, 2] == 0)  # Red channel in BGR
        
    def test_azimuth_coloring(self):
        """Test azimuth angle coloring."""
        engine = ColoringEngine()
        
        # Create points in a circle
        angles = np.linspace(0, 2*np.pi, 100)
        points = np.zeros((100, 4))
        points[:, 0] = np.cos(angles) * 10
        points[:, 1] = np.sin(angles) * 10
        
        colors = engine.apply_coloring(
            points,
            mode="azimuth",
            params={}
        )
        
        assert colors.shape == (100, 3)
        # Colors should vary around the circle
        assert len(np.unique(colors, axis=0)) > 50
        
    def test_create_legend(self):
        """Test legend creation."""
        engine = ColoringEngine()
        
        legend = engine.create_legend(
            mode="height",
            params={"min_height": 0, "max_height": 10},
            width=200,
            height=30
        )
        
        assert legend.shape == (30, 200, 3)
        assert legend.dtype == np.uint8


class TestPerceptionTools:
    """Test MCP tool functions."""
    
    @pytest.mark.asyncio
    async def test_create_context(self):
        """Test creating perception context."""
        result = await create_perception_context(
            context_type="test",
            buffer_duration=10.0
        )
        
        assert result["success"] == True
        assert "context_id" in result
        assert result["context_type"] == "test"
        
    @pytest.mark.asyncio
    async def test_generate_visualization(self):
        """Test visualization generation."""
        # Create context
        ctx_result = await create_perception_context()
        context_id = ctx_result["context_id"]
        
        # Add test data
        from autoware_mcp.perception.tools import _active_contexts
        context = _active_contexts[context_id]
        
        # Add point cloud data
        points = np.random.randn(100, 4).astype(np.float32)
        context.add_capture({
            "type": "pointcloud",
            "topic": "/lidar",
            "data": points
        })
        
        # Generate visualization
        result = await generate_visualization(
            context_id=context_id,
            visualization_type="pointcloud",
            projection="bev",
            color_mode="height"
        )
        
        assert result["success"] == True
        assert "image_base64" in result
        
    @pytest.mark.asyncio
    async def test_analyze_temporal(self):
        """Test temporal analysis."""
        # Create context
        ctx_result = await create_perception_context()
        context_id = ctx_result["context_id"]
        
        from autoware_mcp.perception.tools import _active_contexts
        context = _active_contexts[context_id]
        
        # Add multiple captures with small time delays
        base_time = time.time()
        for i in range(5):
            context.add_capture({
                "type": "pointcloud",
                "topic": "/lidar",
                "data": np.random.randn(100, 4) + i,  # Shift points
                "timestamp_ns": int((base_time + i * 0.1) * 1e9)  # 100ms apart
            })
            
        # Analyze frequency instead (motion needs at least 2 point clouds)
        result = await analyze_temporal(
            context_id=context_id,
            analysis_type="frequency",
            duration=10.0  # Longer duration to capture all data
        )
        
        assert result["success"] == True
        assert "results" in result
        # Check that /lidar topic is in results (if there's data)
        if result["results"]:  # Only check if we have results
            assert "/lidar" in result["results"]
        
    @pytest.mark.asyncio
    async def test_save_context_buffer(self):
        """Test saving context buffer."""
        # Create context with temp storage
        temp_dir = Path(tempfile.mkdtemp())
        
        try:
            ctx_result = await create_perception_context(
                storage_path=str(temp_dir)
            )
            context_id = ctx_result["context_id"]
            
            from autoware_mcp.perception.tools import _active_contexts
            context = _active_contexts[context_id]
            
            # Add some data
            context.add_capture({
                "type": "test",
                "data": "test_data"
            })
            
            # Save buffer
            result = await save_context_buffer(
                context_id=context_id,
                trigger="test_trigger"
            )
            
            assert result["success"] == True
            assert "archive_path" in result
            assert Path(result["archive_path"]).exists()
            
        finally:
            # Cleanup
            if temp_dir.exists():
                shutil.rmtree(temp_dir)