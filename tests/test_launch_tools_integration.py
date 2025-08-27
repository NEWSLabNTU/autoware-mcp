#!/usr/bin/env python3
"""Integration tests for all launch tools."""

import asyncio
import pytest
import time
import os
from pathlib import Path
from unittest.mock import Mock, AsyncMock, patch
import tempfile
import shutil

# Import the tools to test
from src.autoware_mcp.tools.launch_tools import LaunchTools
from src.autoware_mcp.launch_manager import LaunchSessionManager, LaunchGenerator


@pytest.fixture
async def launch_tools():
    """Create LaunchTools instance for testing."""
    # Mock ROS2Manager to avoid ROS2 dependencies in tests
    mock_ros2_manager = Mock()
    mock_ros2_manager.call_service = AsyncMock(return_value={"success": True})
    mock_ros2_manager.list_nodes = AsyncMock(return_value=[])
    mock_ros2_manager.list_topics = AsyncMock(return_value=[])

    tools = LaunchTools(ros2_manager=mock_ros2_manager)
    yield tools

    # Cleanup: stop all sessions
    sessions = await tools.list_launch_sessions()
    for session in sessions:
        try:
            await tools.stop_launch(session["session_id"], force=True)
        except:
            pass


@pytest.fixture
def test_launch_file(tmp_path):
    """Create a test launch file."""
    launch_file = tmp_path / "test_demo.launch.py"
    launch_file.write_text("""#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_talker'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='test_listener'
        )
    ])
""")
    launch_file.chmod(0o755)
    return str(launch_file)


@pytest.fixture
def temp_generated_dir(tmp_path):
    """Create temporary directory for generated files."""
    gen_dir = tmp_path / ".autoware-mcp" / "generated"
    gen_dir.mkdir(parents=True, exist_ok=True)
    (gen_dir / "launches").mkdir(exist_ok=True)
    (gen_dir / "nodes").mkdir(exist_ok=True)
    (gen_dir / "configs").mkdir(exist_ok=True)
    return gen_dir


class TestLaunchSessionManagement:
    """Test launch session management tools."""

    @pytest.mark.asyncio
    async def test_start_and_stop_launch(self, launch_tools, test_launch_file):
        """Test starting and stopping a launch session."""
        # Start launch
        result = await launch_tools.start_launch(test_launch_file)
        assert result["success"] is True
        assert "session_id" in result
        assert "main_pid" in result
        assert "pgid" in result
        assert result["state"] == "running"

        session_id = result["session_id"]

        # Verify session is in list
        sessions = await launch_tools.list_launch_sessions()
        assert len(sessions) > 0
        assert any(s["session_id"] == session_id for s in sessions)

        # Stop launch
        stop_result = await launch_tools.stop_launch(session_id)
        assert stop_result["success"] is True

        # Verify session is no longer in list
        sessions = await launch_tools.list_launch_sessions()
        assert not any(s["session_id"] == session_id for s in sessions)

    @pytest.mark.asyncio
    async def test_get_session_status(self, launch_tools, test_launch_file):
        """Test getting detailed session status."""
        # Start launch
        result = await launch_tools.start_launch(test_launch_file)
        session_id = result["session_id"]

        # Get status
        status = await launch_tools.get_session_status(session_id)
        assert status["success"] is True
        assert status["session_id"] == session_id
        assert status["state"] == "running"
        assert "launch_file" in status
        assert "main_pid" in status
        assert "pgid" in status
        assert "created_at" in status
        assert "is_active" in status

        # Cleanup
        await launch_tools.stop_launch(session_id)

    @pytest.mark.asyncio
    async def test_pause_and_resume_launch(self, launch_tools, test_launch_file):
        """Test pausing and resuming a launch session."""
        # Start launch
        result = await launch_tools.start_launch(test_launch_file)
        session_id = result["session_id"]

        # Pause session
        pause_result = await launch_tools.pause_launch(session_id)
        assert pause_result["success"] is True

        # Check status shows paused
        status = await launch_tools.get_session_status(session_id)
        assert status["state"] == "paused"

        # Resume session
        resume_result = await launch_tools.resume_launch(session_id)
        assert resume_result["success"] is True

        # Check status shows running
        status = await launch_tools.get_session_status(session_id)
        assert status["state"] == "running"

        # Cleanup
        await launch_tools.stop_launch(session_id)

    @pytest.mark.asyncio
    async def test_restart_launch(self, launch_tools, test_launch_file):
        """Test restarting a launch session."""
        # Start launch
        result = await launch_tools.start_launch(test_launch_file)
        old_session_id = result["session_id"]
        old_pid = result["main_pid"]

        # Restart session
        restart_result = await launch_tools.restart_launch(old_session_id)
        assert restart_result["success"] is True
        assert "old_session_id" in restart_result
        assert "new_session_id" in restart_result

        new_session_id = restart_result["new_session_id"]
        assert old_session_id != new_session_id

        # Verify new session is running
        status = await launch_tools.get_session_status(new_session_id)
        assert status["state"] == "running"
        assert status["main_pid"] != old_pid

        # Cleanup
        await launch_tools.stop_launch(new_session_id)

    @pytest.mark.asyncio
    async def test_get_session_logs(self, launch_tools, test_launch_file):
        """Test retrieving session logs."""
        # Start launch
        result = await launch_tools.start_launch(test_launch_file)
        session_id = result["session_id"]

        # Give it time to generate some logs
        await asyncio.sleep(2)

        # Get stdout logs
        logs = await launch_tools.get_session_logs(
            session_id, lines=50, stream="stdout"
        )
        assert logs["success"] is True
        assert logs["session_id"] == session_id
        assert logs["stream"] == "stdout"
        # Content might be empty if nodes haven't produced output yet
        assert "content" in logs

        # Get stderr logs
        logs_err = await launch_tools.get_session_logs(
            session_id, lines=50, stream="stderr"
        )
        assert logs_err["success"] is True
        assert logs_err["stream"] == "stderr"

        # Cleanup
        await launch_tools.stop_launch(session_id)

    @pytest.mark.asyncio
    async def test_cleanup_orphans(self, launch_tools):
        """Test cleanup of orphaned sessions."""
        # This should not fail even if no orphans exist
        result = await launch_tools.cleanup_orphans()
        assert result["success"] is True
        assert "cleaned_up_count" in result

    @pytest.mark.asyncio
    async def test_get_launch_errors(self, launch_tools, test_launch_file):
        """Test getting error diagnostics from a session."""
        # Start launch
        result = await launch_tools.start_launch(test_launch_file)
        session_id = result["session_id"]

        # Get errors (should be none for successful launch)
        errors = await launch_tools.get_launch_errors(session_id)
        assert errors["success"] is True
        assert errors["session_id"] == session_id
        assert errors["state"] == "running"

        # Cleanup
        await launch_tools.stop_launch(session_id)


class TestLaunchGeneration:
    """Test launch file and configuration generation tools."""

    @pytest.mark.asyncio
    async def test_generate_launch_file(self, launch_tools, temp_generated_dir):
        """Test generating a launch file."""
        # Mock the generator to use temp directory
        with patch.object(
            launch_tools.generator, "launches_dir", temp_generated_dir / "launches"
        ):
            result = await launch_tools.generate_launch_file(
                name="test_generated", components=["talker", "listener"], template=None
            )

            assert result["success"] is True
            assert "file_path" in result
            assert "version" in result
            assert result["components"] == ["talker", "listener"]

            # Verify file exists
            assert Path(result["file_path"]).exists()

            # Verify content
            content = Path(result["file_path"]).read_text()
            assert "def generate_launch_description()" in content
            assert "talker" in content
            assert "listener" in content

    @pytest.mark.asyncio
    async def test_generate_launch_with_template(
        self, launch_tools, temp_generated_dir
    ):
        """Test generating launch file with perception template."""
        with patch.object(
            launch_tools.generator, "launches_dir", temp_generated_dir / "launches"
        ):
            result = await launch_tools.generate_launch_file(
                name="test_perception",
                components=["lidar", "camera"],
                template="perception_pipeline",
            )

            assert result["success"] is True
            assert Path(result["file_path"]).exists()

            # Check perception-specific content
            content = Path(result["file_path"]).read_text()
            assert "perception" in content.lower()

    @pytest.mark.asyncio
    async def test_validate_launch_file(self, launch_tools, test_launch_file):
        """Test validating launch file syntax."""
        result = await launch_tools.validate_launch_file(test_launch_file)
        assert result["success"] is True
        assert result["message"] == "Launch file is valid"

    @pytest.mark.asyncio
    async def test_validate_invalid_launch_file(self, launch_tools, tmp_path):
        """Test validating an invalid launch file."""
        bad_file = tmp_path / "bad.launch.py"
        bad_file.write_text("this is not valid python code {{{")

        result = await launch_tools.validate_launch_file(str(bad_file))
        assert result["success"] is False
        assert "error" in result

    @pytest.mark.asyncio
    async def test_generate_node_config(self, launch_tools, temp_generated_dir):
        """Test generating node configuration."""
        with patch.object(
            launch_tools.generator, "configs_dir", temp_generated_dir / "configs"
        ):
            result = await launch_tools.generate_node_config(
                node_name="test_node",
                parameters={"param1": "value1", "param2": 123},
                format="yaml",
            )

            assert result["success"] is True
            assert "file_path" in result
            assert Path(result["file_path"]).exists()

            # Verify YAML content
            import yaml

            with open(result["file_path"], "r") as f:
                config = yaml.safe_load(f)
                assert config["/test_node"]["ros__parameters"]["param1"] == "value1"
                assert config["/test_node"]["ros__parameters"]["param2"] == 123

    @pytest.mark.asyncio
    async def test_generate_node_config_json(self, launch_tools, temp_generated_dir):
        """Test generating node configuration in JSON format."""
        with patch.object(
            launch_tools.generator, "configs_dir", temp_generated_dir / "configs"
        ):
            result = await launch_tools.generate_node_config(
                node_name="test_node", parameters={"param1": "value1"}, format="json"
            )

            assert result["success"] is True
            import json

            with open(result["file_path"], "r") as f:
                config = json.load(f)
                assert config["/test_node"]["ros__parameters"]["param1"] == "value1"

    @pytest.mark.asyncio
    async def test_generate_custom_node(self, launch_tools, temp_generated_dir):
        """Test generating custom node template."""
        with patch.object(
            launch_tools.generator, "nodes_dir", temp_generated_dir / "nodes"
        ):
            result = await launch_tools.generate_custom_node(
                name="test_custom_node", language="python", node_type="basic"
            )

            assert result["success"] is True
            assert "file_path" in result
            assert Path(result["file_path"]).exists()

            # Verify Python node content
            content = Path(result["file_path"]).read_text()
            assert "rclpy" in content
            assert "class TestCustomNodeNode" in content  # Class name follows pattern
            assert "def main(args=None):" in content

    @pytest.mark.asyncio
    async def test_generate_cpp_node(self, launch_tools, temp_generated_dir):
        """Test generating C++ node template."""
        with patch.object(
            launch_tools.generator, "nodes_dir", temp_generated_dir / "nodes"
        ):
            result = await launch_tools.generate_custom_node(
                name="test_cpp_node", language="cpp", node_type="basic"
            )

            assert result["success"] is True
            content = Path(result["file_path"]).read_text()
            assert "#include <rclcpp/rclcpp.hpp>" in content
            assert "class TestCppNode" in content

    @pytest.mark.asyncio
    async def test_test_launch_file(self, launch_tools, test_launch_file):
        """Test dry-run validation of launch file."""
        result = await launch_tools.test_launch_file(test_launch_file, dry_run=True)
        assert result["success"] is True
        assert result["dry_run"] is True
        assert result["message"] == "Launch file validation passed"

    @pytest.mark.asyncio
    async def test_list_generated_files(self, launch_tools, temp_generated_dir):
        """Test listing generated files."""
        with patch.object(launch_tools.generator, "generated_dir", temp_generated_dir):
            # Generate some files first
            with patch.object(
                launch_tools.generator, "launches_dir", temp_generated_dir / "launches"
            ):
                await launch_tools.generate_launch_file(
                    name="test1", components=["node1"]
                )

            # List files
            files = await launch_tools.list_generated_files()
            assert "launches" in files
            assert "nodes" in files
            assert "configs" in files
            assert len(files["launches"]) > 0


class TestEdgeCases:
    """Test edge cases and error handling."""

    @pytest.mark.asyncio
    async def test_stop_nonexistent_session(self, launch_tools):
        """Test stopping a non-existent session."""
        result = await launch_tools.stop_launch("nonexistent-session-id")
        assert result["success"] is False
        assert "error" in result

    @pytest.mark.asyncio
    async def test_get_status_nonexistent_session(self, launch_tools):
        """Test getting status of non-existent session."""
        result = await launch_tools.get_session_status("nonexistent-session-id")
        assert result["success"] is False
        assert "error" in result

    @pytest.mark.asyncio
    async def test_start_nonexistent_launch_file(self, launch_tools):
        """Test starting a non-existent launch file."""
        result = await launch_tools.start_launch("/nonexistent/file.launch.py")
        assert result["success"] is False
        assert "error" in result

    @pytest.mark.asyncio
    async def test_multiple_sessions(self, launch_tools, test_launch_file):
        """Test managing multiple concurrent sessions."""
        sessions = []

        try:
            # Start multiple sessions
            for i in range(3):
                result = await launch_tools.start_launch(test_launch_file)
                assert result["success"] is True
                sessions.append(result["session_id"])

            # List should show all sessions
            session_list = await launch_tools.list_launch_sessions()
            assert len(session_list) >= 3

            # Each should have unique session ID
            assert len(set(sessions)) == 3

        finally:
            # Cleanup all sessions
            for session_id in sessions:
                await launch_tools.stop_launch(session_id)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
