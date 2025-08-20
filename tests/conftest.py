"""Shared pytest fixtures and configuration."""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock


@pytest.fixture
def mock_ros2_manager():
    """Create a mock ROS2 manager for testing."""
    manager = Mock()
    manager.list_nodes = AsyncMock(return_value=[])
    manager.list_topics = AsyncMock(return_value=[])
    manager.list_services = AsyncMock(return_value=[])
    manager.get_node_info = AsyncMock(return_value={})
    manager.get_topic_info = AsyncMock(return_value={})
    manager.verify_environment = AsyncMock(return_value={"ready": True})
    manager.check_autoware_status = AsyncMock(return_value={"status": "operational"})
    return manager


@pytest.fixture
def mock_health_monitor():
    """Create a mock health monitor for testing."""
    monitor = Mock()
    monitor.get_complete_health_status = AsyncMock(
        return_value={
            "overall_status": "healthy",
            "timestamp": "2025-01-19T12:00:00",
            "uptime_seconds": 100.0,
            "system": {},
            "ros2": {},
            "autoware": {},
        }
    )
    monitor.get_system_status = AsyncMock(return_value={})
    monitor.get_heartbeat_status = AsyncMock(return_value={"alive": True})
    return monitor


@pytest.fixture
def mock_ad_api():
    """Create a mock AD API for testing."""
    api = Mock()
    api.set_operation_mode = AsyncMock(return_value={"success": True})
    api.monitor_operation_mode = AsyncMock(return_value={})
    api.set_route = AsyncMock(return_value={"success": True})
    api.get_current_route = AsyncMock(return_value={})
    api.initialize_localization = AsyncMock(return_value={"success": True})
    api.monitor_localization_state = AsyncMock(return_value={})
    api.request_mrm = AsyncMock(return_value={"success": True})
    api.list_mrm_behaviors = AsyncMock(return_value=[])
    api.monitor_mrm_state = AsyncMock(return_value={})
    api.get_vehicle_state = AsyncMock(return_value={})
    api.monitor_diagnostics = AsyncMock(return_value={})
    api.reset_diagnostics = AsyncMock(return_value={"success": True})
    return api


@pytest.fixture
def event_loop():
    """Create an event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()
