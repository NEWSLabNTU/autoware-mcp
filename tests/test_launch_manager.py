"""Tests for launch session management."""

import pytest
import asyncio
import tempfile
import shutil
from pathlib import Path
from unittest.mock import Mock, AsyncMock, patch, MagicMock
import signal
import os

from autoware_mcp.launch_manager import (
    LaunchSessionManager,
    LaunchSession,
    SessionState,
    ProcessTracker,
    CleanupManager,
    LaunchGenerator,
)


class TestLaunchSession:
    """Test LaunchSession class."""

    def test_session_creation(self):
        """Test creating a new launch session."""
        session = LaunchSession(launch_file="/path/to/launch.py")

        assert session.session_id
        assert session.launch_file == "/path/to/launch.py"
        assert session.state == SessionState.INITIALIZED
        assert session.main_pid is None
        assert session.pgid is None
        assert session.nodes == []

    def test_state_update(self):
        """Test updating session state."""
        session = LaunchSession()

        session.update_state(SessionState.RUNNING)
        assert session.state == SessionState.RUNNING
        assert session.error_message is None

        session.update_state(SessionState.ERROR, "Test error")
        assert session.state == SessionState.ERROR
        assert session.error_message == "Test error"

    def test_is_active(self):
        """Test checking if session is active."""
        session = LaunchSession()

        # Inactive states
        assert not session.is_active()

        session.state = SessionState.TERMINATED
        assert not session.is_active()

        session.state = SessionState.ERROR
        assert not session.is_active()

        # Active states
        session.state = SessionState.STARTING
        assert session.is_active()

        session.state = SessionState.RUNNING
        assert session.is_active()

        session.state = SessionState.PAUSED
        assert session.is_active()

    def test_status_dict(self):
        """Test getting session status as dictionary."""
        session = LaunchSession(launch_file="/test/launch.py")
        session.main_pid = 12345
        session.pgid = 12340

        status = session.get_status_dict()

        assert status["session_id"] == session.session_id
        assert status["launch_file"] == "/test/launch.py"
        assert status["state"] == "initialized"
        assert status["main_pid"] == 12345
        assert status["pgid"] == 12340
        assert status["node_count"] == 0
        assert status["is_active"] is False


class TestProcessTracker:
    """Test ProcessTracker class."""

    def setup_method(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        self.tracker = ProcessTracker(Path(self.temp_dir))

    def teardown_method(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_instance_directory_creation(self):
        """Test that instance directory is created."""
        assert self.tracker.instance_dir.exists()
        assert "instance_" in self.tracker.instance_dir.name

    def test_write_and_read_pid(self):
        """Test writing and reading PID files."""
        session_id = "test-session"
        test_pid = 12345

        self.tracker.write_pid_file(session_id, test_pid)

        read_pid = self.tracker.read_pid(session_id)
        assert read_pid == test_pid

    def test_write_and_read_pgid(self):
        """Test writing and reading PGID files."""
        session_id = "test-session"
        test_pgid = 12340

        self.tracker.write_pgid_file(session_id, test_pgid)

        read_pgid = self.tracker.read_pgid(session_id)
        assert read_pgid == test_pgid

    def test_read_nonexistent_pid(self):
        """Test reading PID that doesn't exist."""
        assert self.tracker.read_pid("nonexistent") is None

    def test_read_nonexistent_pgid(self):
        """Test reading PGID that doesn't exist."""
        assert self.tracker.read_pgid("nonexistent") is None

    @patch("autoware_mcp.launch_manager.process_tracker.psutil.Process")
    def test_is_process_alive(self, mock_process):
        """Test checking if process is alive."""
        mock_proc_instance = Mock()
        mock_proc_instance.is_running.return_value = True
        mock_process.return_value = mock_proc_instance

        assert self.tracker.is_process_alive(12345)

        mock_proc_instance.is_running.return_value = False
        assert not self.tracker.is_process_alive(12345)

    def test_signal_process_group(self):
        """Test signaling process group."""
        with patch("os.killpg") as mock_killpg:
            assert self.tracker.signal_process_group(12340, signal.SIGTERM)
            mock_killpg.assert_called_once_with(12340, signal.SIGTERM)

        with patch("os.killpg", side_effect=ProcessLookupError):
            assert not self.tracker.signal_process_group(99999, signal.SIGTERM)


class TestCleanupManager:
    """Test CleanupManager class."""

    def setup_method(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        self.tracker = ProcessTracker(Path(self.temp_dir))
        self.cleanup = CleanupManager(self.tracker)

    def teardown_method(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_register_session(self):
        """Test registering a session for cleanup."""
        session = LaunchSession()
        self.cleanup.register_session(session)

        assert session.session_id in self.cleanup.sessions

    def test_unregister_session(self):
        """Test unregistering a session."""
        session = LaunchSession()
        self.cleanup.register_session(session)
        self.cleanup.unregister_session(session.session_id)

        assert session.session_id not in self.cleanup.sessions

    @patch("os.killpg")
    def test_cleanup_session_graceful(self, mock_killpg):
        """Test graceful cleanup of session."""
        session_id = "test-session"
        self.tracker.write_pgid_file(session_id, 12340)

        # Simulate process group no longer exists after SIGTERM
        mock_killpg.side_effect = [None, ProcessLookupError]

        assert self.cleanup.cleanup_session(session_id, force=False, timeout=0.1)

        # Should have called SIGTERM
        assert mock_killpg.call_count >= 1
        assert mock_killpg.call_args_list[0][0] == (12340, signal.SIGTERM)

    @patch("os.killpg")
    def test_cleanup_session_force(self, mock_killpg):
        """Test forced cleanup of session."""
        session_id = "test-session"
        self.tracker.write_pgid_file(session_id, 12340)

        assert self.cleanup.cleanup_session(session_id, force=True)

        # Should have called SIGKILL immediately
        mock_killpg.assert_called_once_with(12340, signal.SIGKILL)


class TestLaunchSessionManager:
    """Test LaunchSessionManager class."""

    def setup_method(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        self.manager = LaunchSessionManager()
        self.manager.tracker = ProcessTracker(Path(self.temp_dir))
        self.manager.cleanup_manager = CleanupManager(self.manager.tracker)

    def teardown_method(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    @pytest.mark.asyncio
    async def test_start_launch_file_not_found(self):
        """Test starting launch with non-existent file."""
        result = await self.manager.start_launch("/nonexistent/launch.py")

        assert not result["success"]
        assert "not found" in result["error"]

    @pytest.mark.asyncio
    @patch("subprocess.Popen")
    @patch("os.setsid")
    @patch("os.getpgid")
    async def test_start_launch_success(self, mock_getpgid, mock_setsid, mock_popen):
        """Test successful launch start."""
        # Create a temporary launch file
        launch_file = Path(self.temp_dir) / "test.launch.py"
        launch_file.write_text("# Test launch file")

        # Mock process
        mock_process = Mock()
        mock_process.pid = 12345
        mock_process.poll.return_value = None  # Process is running
        mock_popen.return_value = mock_process
        mock_getpgid.return_value = 12340

        result = await self.manager.start_launch(str(launch_file))

        assert result["success"]
        assert "session_id" in result
        assert result["main_pid"] == 12345
        assert result["pgid"] == 12340

        # Check session is tracked
        assert result["session_id"] in self.manager.sessions

    @pytest.mark.asyncio
    async def test_stop_launch_not_found(self):
        """Test stopping non-existent session."""
        result = await self.manager.stop_launch("nonexistent")

        assert not result["success"]
        assert "not found" in result["error"]

    @pytest.mark.asyncio
    @patch("autoware_mcp.launch_manager.cleanup.CleanupManager.cleanup_session")
    async def test_stop_launch_success(self, mock_cleanup):
        """Test successful launch stop."""
        # Create a session
        session = LaunchSession(launch_file="/test/launch.py")
        session.state = SessionState.RUNNING
        self.manager.sessions[session.session_id] = session

        mock_cleanup.return_value = True

        result = await self.manager.stop_launch(session.session_id)

        assert result["success"]
        assert session.session_id not in self.manager.sessions

    @pytest.mark.asyncio
    async def test_pause_launch(self):
        """Test pausing a launch session."""
        session = LaunchSession()
        session.state = SessionState.RUNNING
        session.pgid = 12340
        self.manager.sessions[session.session_id] = session

        with patch.object(
            self.manager.tracker, "signal_process_group", return_value=True
        ):
            result = await self.manager.pause_launch(session.session_id)

            assert result["success"]
            assert session.state == SessionState.PAUSED

    @pytest.mark.asyncio
    async def test_resume_launch(self):
        """Test resuming a paused session."""
        session = LaunchSession()
        session.state = SessionState.PAUSED
        session.pgid = 12340
        self.manager.sessions[session.session_id] = session

        with patch.object(
            self.manager.tracker, "signal_process_group", return_value=True
        ):
            result = await self.manager.resume_launch(session.session_id)

            assert result["success"]
            assert session.state == SessionState.RUNNING

    @pytest.mark.asyncio
    async def test_list_sessions(self):
        """Test listing all sessions."""
        # Add some sessions
        session1 = LaunchSession(launch_file="/test1.py")
        session2 = LaunchSession(launch_file="/test2.py")

        self.manager.sessions[session1.session_id] = session1
        self.manager.sessions[session2.session_id] = session2

        sessions = await self.manager.list_sessions()

        assert len(sessions) == 2
        assert any(s["launch_file"] == "/test1.py" for s in sessions)
        assert any(s["launch_file"] == "/test2.py" for s in sessions)

    @pytest.mark.asyncio
    async def test_get_session_status(self):
        """Test getting session status."""
        session = LaunchSession(launch_file="/test.py")
        session.main_pid = 12345
        session.pgid = 12340
        self.manager.sessions[session.session_id] = session

        with patch.object(self.manager.tracker, "is_process_alive", return_value=True):
            with patch.object(
                self.manager.tracker,
                "get_process_children",
                return_value=[12346, 12347],
            ):
                result = await self.manager.get_session_status(session.session_id)

                assert result["success"]
                assert result["launch_file"] == "/test.py"
                assert result["main_pid"] == 12345
                assert result["pgid"] == 12340
                assert result["process_alive"]
                assert result["child_process_count"] == 2


class TestLaunchGenerator:
    """Test LaunchGenerator class."""

    def setup_method(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        self.generator = LaunchGenerator(Path(self.temp_dir))

    def teardown_method(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_generate_launch_file(self):
        """Test generating a launch file."""
        result = self.generator.generate_launch_file(
            name="test_launch", components=["node1", "node2"]
        )

        assert result["success"]
        assert "file_path" in result
        assert Path(result["file_path"]).exists()
        assert result["version"] == 1

        # Generate another version
        result2 = self.generator.generate_launch_file(
            name="test_launch", components=["node3"]
        )

        assert result2["version"] == 2

    def test_generate_node_config_yaml(self):
        """Test generating YAML config file."""
        result = self.generator.generate_node_config(
            node_name="test_node",
            parameters={"param1": "value1", "param2": 42},
            format="yaml",
        )

        assert result["success"]
        assert Path(result["file_path"]).exists()
        assert result["format"] == "yaml"
        assert ".yaml" in result["filename"]

    def test_generate_node_config_json(self):
        """Test generating JSON config file."""
        result = self.generator.generate_node_config(
            node_name="test_node",
            parameters={"param1": "value1", "param2": 42},
            format="json",
        )

        assert result["success"]
        assert Path(result["file_path"]).exists()
        assert result["format"] == "json"
        assert ".json" in result["filename"]

    def test_generate_custom_node_python(self):
        """Test generating Python node."""
        result = self.generator.generate_custom_node(
            name="test_node",
            language="python",
            node_type="basic",
            interfaces=["topic1", "topic2"],
        )

        assert result["success"]
        assert Path(result["file_path"]).exists()
        assert result["language"] == "python"
        assert ".py" in result["filename"]

        # Check file is executable
        assert os.access(result["file_path"], os.X_OK)

    def test_generate_custom_node_cpp(self):
        """Test generating C++ node."""
        result = self.generator.generate_custom_node(
            name="test_node", language="cpp", node_type="basic"
        )

        assert result["success"]
        assert Path(result["file_path"]).exists()
        assert result["language"] == "cpp"
        assert ".cpp" in result["filename"]

    def test_validate_launch_file_valid(self):
        """Test validating a valid launch file."""
        launch_file = Path(self.temp_dir) / "valid.launch.py"
        launch_file.write_text("""
def generate_launch_description():
    return []
""")

        result = self.generator.validate_launch_file(str(launch_file))

        assert result["success"]
        assert "valid" in result["message"].lower()

    def test_validate_launch_file_invalid(self):
        """Test validating an invalid launch file."""
        launch_file = Path(self.temp_dir) / "invalid.launch.py"
        launch_file.write_text("""
def invalid_syntax(:
    return []
""")

        result = self.generator.validate_launch_file(str(launch_file))

        assert not result["success"]
        assert "Syntax error" in result["error"]

    def test_validate_launch_file_missing_function(self):
        """Test validating launch file without required function."""
        launch_file = Path(self.temp_dir) / "missing.launch.py"
        launch_file.write_text("""
def some_other_function():
    return []
""")

        result = self.generator.validate_launch_file(str(launch_file))

        assert not result["success"]
        assert "generate_launch_description" in result["error"]

    def test_list_generated_files(self):
        """Test listing generated files."""
        # Generate some files
        self.generator.generate_launch_file("test1", ["node1"])
        self.generator.generate_node_config("node1", {"p": 1})
        self.generator.generate_custom_node("custom1", "python")

        files = self.generator.list_generated_files()

        assert len(files["launches"]) == 1
        assert len(files["configs"]) == 1
        assert len(files["nodes"]) == 1
