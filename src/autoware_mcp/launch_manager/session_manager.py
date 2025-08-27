"""Main launch session manager implementation."""

import os
import signal
import logging
import subprocess
import asyncio
import time
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
from concurrent.futures import ThreadPoolExecutor

from .session import LaunchSession, SessionState
from .process_tracker import ProcessTracker
from .cleanup import CleanupManager

logger = logging.getLogger(__name__)


class LaunchSessionManager:
    """Manages ROS2 launch sessions with comprehensive lifecycle control."""

    def __init__(self, ros2_manager=None):
        """Initialize launch session manager.

        Args:
            ros2_manager: Optional ROS2Manager instance for ROS2 operations
        """
        self.ros2_manager = ros2_manager
        self.tracker = ProcessTracker()
        self.cleanup_manager = CleanupManager(self.tracker)
        self.sessions: Dict[str, LaunchSession] = {}
        self.executor = ThreadPoolExecutor(max_workers=5)

        # Recover any existing sessions
        self._recover_sessions()

    def _recover_sessions(self):
        """Recover sessions from previous MCP instance."""
        recovered = self.cleanup_manager.recover_sessions()
        for session in recovered:
            self.sessions[session.session_id] = session

        if recovered:
            logger.info(f"Recovered {len(recovered)} sessions from previous instance")

    async def start_launch(
        self,
        launch_file: str,
        parameters: Optional[Dict[str, Any]] = None,
        launch_args: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Start a ROS2 launch file.

        Args:
            launch_file: Path to the launch file
            parameters: Optional parameters to pass to launch
            launch_args: Optional command line arguments

        Returns:
            Session information including ID and status
        """
        # Validate launch file exists
        launch_path = Path(launch_file)
        if not launch_path.exists():
            return {"success": False, "error": f"Launch file not found: {launch_file}"}

        # Create new session
        session = LaunchSession(
            launch_file=str(launch_path.absolute()), parameters=parameters or {}
        )

        # Register session
        self.sessions[session.session_id] = session
        self.cleanup_manager.register_session(session)

        # Create logs directory
        logs_dir = self.tracker.base_dir / "logs" / f"session_{session.session_id}"
        logs_dir.mkdir(parents=True, exist_ok=True)
        session.logs_path = str(logs_dir)

        # Build launch command
        cmd = ["ros2", "launch", str(launch_path)]

        # Add launch arguments
        if launch_args:
            cmd.extend(launch_args)

        # Add parameters as launch arguments
        if parameters:
            for key, value in parameters.items():
                cmd.append(f"{key}:={value}")

        # Update state to starting
        session.update_state(SessionState.STARTING)

        try:
            # Start launch process with new process group
            stdout_file = open(logs_dir / "stdout.log", "w")
            stderr_file = open(logs_dir / "stderr.log", "w")

            process = subprocess.Popen(
                cmd,
                stdout=stdout_file,
                stderr=stderr_file,
                preexec_fn=os.setsid,  # Create new process group
                env={
                    **os.environ,
                    "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
                },
            )

            # Record PID and PGID
            session.main_pid = process.pid
            session.pgid = os.getpgid(process.pid)

            # Write PID/PGID files
            self.tracker.write_pid_file(session.session_id, session.main_pid)
            self.tracker.write_pgid_file(session.session_id, session.pgid)

            # Wait a bit to check if process started successfully
            await asyncio.sleep(2.0)

            if process.poll() is not None:
                # Process already terminated
                session.update_state(
                    SessionState.ERROR, "Launch process terminated immediately"
                )
                return {
                    "success": False,
                    "session_id": session.session_id,
                    "error": "Launch process terminated immediately",
                    "logs_path": session.logs_path,
                }

            # Update state to running
            session.update_state(SessionState.RUNNING)

            # Start monitoring in background
            self.executor.submit(self._monitor_session, session.session_id, process)

            return {
                "success": True,
                "session_id": session.session_id,
                "main_pid": session.main_pid,
                "pgid": session.pgid,
                "state": session.state.value,
                "logs_path": session.logs_path,
            }

        except Exception as e:
            session.update_state(SessionState.ERROR, str(e))
            logger.error(f"Failed to start launch file {launch_file}: {e}")
            return {"success": False, "session_id": session.session_id, "error": str(e)}

    def _monitor_session(self, session_id: str, process: subprocess.Popen):
        """Monitor a launch session in background thread."""
        session = self.sessions.get(session_id)
        if not session:
            return

        logger.info(f"Starting monitor for session {session_id}")

        while True:
            # Check if process is still running
            poll_result = process.poll()

            if poll_result is not None:
                # Process terminated
                if session.state == SessionState.STOPPING:
                    session.update_state(SessionState.TERMINATED)
                else:
                    session.update_state(
                        SessionState.ERROR,
                        f"Process terminated with code {poll_result}",
                    )

                logger.info(f"Session {session_id} terminated with code {poll_result}")
                break

            # Update node information if ROS2 manager available
            if self.ros2_manager:
                try:
                    # Get child processes for node tracking
                    children = self.tracker.get_process_children(session.main_pid)
                    # Update session with node information
                    # This would require ROS2 introspection
                except Exception as e:
                    logger.debug(f"Failed to update nodes for {session_id}: {e}")

            time.sleep(5)  # Check every 5 seconds

    async def stop_launch(
        self, session_id: str, force: bool = False, timeout: float = 10.0
    ) -> Dict[str, Any]:
        """Stop a launch session.

        Args:
            session_id: Session ID to stop
            force: If True, use SIGKILL immediately
            timeout: Timeout for graceful shutdown

        Returns:
            Status of the stop operation
        """
        session = self.sessions.get(session_id)
        if not session:
            return {"success": False, "error": f"Session {session_id} not found"}

        if session.state == SessionState.TERMINATED:
            return {"success": True, "message": "Session already terminated"}

        # Update state
        session.update_state(SessionState.STOPPING)

        # Perform cleanup
        success = self.cleanup_manager.cleanup_session(session_id, force, timeout)

        if success:
            session.update_state(SessionState.TERMINATED)
            del self.sessions[session_id]

            return {
                "success": True,
                "message": f"Session {session_id} stopped successfully",
            }
        else:
            return {"success": False, "error": f"Failed to stop session {session_id}"}

    async def pause_launch(self, session_id: str) -> Dict[str, Any]:
        """Pause a launch session using SIGSTOP.

        Args:
            session_id: Session ID to pause

        Returns:
            Status of the pause operation
        """
        session = self.sessions.get(session_id)
        if not session:
            return {"success": False, "error": f"Session {session_id} not found"}

        if session.state != SessionState.RUNNING:
            return {
                "success": False,
                "error": f"Session is not running (state: {session.state.value})",
            }

        if session.pgid:
            success = self.tracker.signal_process_group(session.pgid, signal.SIGSTOP)
            if success:
                session.update_state(SessionState.PAUSED)
                return {"success": True, "message": f"Session {session_id} paused"}

        return {"success": False, "error": f"Failed to pause session {session_id}"}

    async def resume_launch(self, session_id: str) -> Dict[str, Any]:
        """Resume a paused launch session using SIGCONT.

        Args:
            session_id: Session ID to resume

        Returns:
            Status of the resume operation
        """
        session = self.sessions.get(session_id)
        if not session:
            return {"success": False, "error": f"Session {session_id} not found"}

        if session.state != SessionState.PAUSED:
            return {
                "success": False,
                "error": f"Session is not paused (state: {session.state.value})",
            }

        session.update_state(SessionState.RESUMING)

        if session.pgid:
            success = self.tracker.signal_process_group(session.pgid, signal.SIGCONT)
            if success:
                session.update_state(SessionState.RUNNING)
                return {"success": True, "message": f"Session {session_id} resumed"}

        session.update_state(SessionState.ERROR, "Failed to resume")
        return {"success": False, "error": f"Failed to resume session {session_id}"}

    async def restart_launch(self, session_id: str) -> Dict[str, Any]:
        """Restart a launch session (stop and start).

        Args:
            session_id: Session ID to restart

        Returns:
            Status of the restart operation with new session ID
        """
        session = self.sessions.get(session_id)
        if not session:
            return {"success": False, "error": f"Session {session_id} not found"}

        # Save launch configuration
        launch_file = session.launch_file
        parameters = session.parameters

        # Stop the session
        stop_result = await self.stop_launch(session_id)
        if not stop_result["success"]:
            return {
                "success": False,
                "error": f"Failed to stop session: {stop_result.get('error')}",
            }

        # Start new session with same configuration
        start_result = await self.start_launch(launch_file, parameters)

        if start_result["success"]:
            return {
                "success": True,
                "old_session_id": session_id,
                "new_session_id": start_result["session_id"],
                "message": "Session restarted successfully",
            }
        else:
            return {
                "success": False,
                "error": f"Failed to restart: {start_result.get('error')}",
            }

    async def list_sessions(self) -> List[Dict[str, Any]]:
        """List all launch sessions.

        Returns:
            List of session information
        """
        sessions = []
        for session in self.sessions.values():
            sessions.append(session.get_status_dict())

        return sessions

    async def get_session_status(self, session_id: str) -> Dict[str, Any]:
        """Get detailed status of a session.

        Args:
            session_id: Session ID to query

        Returns:
            Detailed session status
        """
        session = self.sessions.get(session_id)
        if not session:
            return {"success": False, "error": f"Session {session_id} not found"}

        status = session.get_status_dict()

        # Add process alive status
        if session.main_pid:
            status["process_alive"] = self.tracker.is_process_alive(session.main_pid)

        # Add children count
        if session.main_pid:
            children = self.tracker.get_process_children(session.main_pid)
            status["child_process_count"] = len(children)

        return {"success": True, **status}

    async def get_session_logs(
        self, session_id: str, lines: int = 100, stream: str = "stdout"
    ) -> Dict[str, Any]:
        """Get logs from a session.

        Args:
            session_id: Session ID
            lines: Number of lines to retrieve
            stream: Log stream (stdout or stderr)

        Returns:
            Log content
        """
        session = self.sessions.get(session_id)
        if not session or not session.logs_path:
            return {
                "success": False,
                "error": f"Session {session_id} not found or no logs available",
            }

        log_file = Path(session.logs_path) / f"{stream}.log"
        if not log_file.exists():
            return {"success": False, "error": f"Log file not found: {log_file}"}

        try:
            # Read last N lines
            with open(log_file) as f:
                all_lines = f.readlines()
                last_lines = all_lines[-lines:] if len(all_lines) > lines else all_lines

            return {
                "success": True,
                "session_id": session_id,
                "stream": stream,
                "lines": len(last_lines),
                "content": "".join(last_lines),
            }
        except Exception as e:
            return {"success": False, "error": f"Failed to read logs: {e}"}

    async def cleanup_orphans(self) -> Dict[str, Any]:
        """Clean up orphaned sessions from stale MCP instances.

        Returns:
            Information about cleaned up sessions
        """
        cleaned_up = self.cleanup_manager.cleanup_orphaned_sessions()

        return {
            "success": True,
            "cleaned_up_count": len(cleaned_up),
            "sessions": cleaned_up,
        }
