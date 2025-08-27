"""Cleanup and recovery mechanisms for launch sessions."""

import os
import signal
import logging
import time
from pathlib import Path
from typing import List, Optional, Dict, Any
import atexit
import threading

from .process_tracker import ProcessTracker
from .session import LaunchSession, SessionState

logger = logging.getLogger(__name__)


class CleanupManager:
    """Manages cleanup of launch sessions and orphaned processes."""

    def __init__(self, process_tracker: Optional[ProcessTracker] = None):
        """Initialize cleanup manager.

        Args:
            process_tracker: ProcessTracker instance to use
        """
        self.tracker = process_tracker or ProcessTracker()
        self.sessions: Dict[str, LaunchSession] = {}
        self._shutdown_event = threading.Event()
        self._registered_handlers = False
        self._register_cleanup_handlers()

    def _register_cleanup_handlers(self):
        """Register cleanup handlers for graceful shutdown."""
        if self._registered_handlers:
            return

        # Register atexit handler
        atexit.register(self._cleanup_all_sessions)

        # Register signal handlers
        for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
            try:
                signal.signal(sig, self._signal_handler)
            except (ValueError, OSError):
                # Signal may not be available on this platform
                pass

        self._registered_handlers = True
        logger.info("Registered cleanup handlers")

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}, initiating cleanup")
        self._shutdown_event.set()
        self._cleanup_all_sessions()

    def register_session(self, session: LaunchSession):
        """Register a session for cleanup tracking."""
        self.sessions[session.session_id] = session
        logger.debug(f"Registered session {session.session_id} for cleanup")

    def unregister_session(self, session_id: str):
        """Unregister a session from cleanup tracking."""
        if session_id in self.sessions:
            del self.sessions[session_id]
            logger.debug(f"Unregistered session {session_id}")

    def _cleanup_all_sessions(self):
        """Clean up all active sessions on shutdown."""
        logger.info(f"Cleaning up {len(self.sessions)} active sessions")

        for session_id, session in list(self.sessions.items()):
            try:
                self.cleanup_session(session_id, force=False)
            except Exception as e:
                logger.error(f"Failed to cleanup session {session_id}: {e}")

    def cleanup_session(
        self, session_id: str, force: bool = False, timeout: float = 10.0
    ) -> bool:
        """Clean up a specific session.

        Args:
            session_id: Session ID to clean up
            force: If True, use SIGKILL immediately
            timeout: Timeout for graceful shutdown

        Returns:
            True if cleanup was successful
        """
        logger.info(f"Cleaning up session {session_id} (force={force})")

        # Get PGID for the session
        pgid = self.tracker.read_pgid(session_id)
        if not pgid:
            logger.warning(f"No PGID found for session {session_id}")
            # Try to get PID instead
            pid = self.tracker.read_pid(session_id)
            if pid and self.tracker.is_process_alive(pid):
                try:
                    os.kill(pid, signal.SIGKILL if force else signal.SIGTERM)
                except ProcessLookupError:
                    pass
        else:
            # Send signal to process group
            sig = signal.SIGKILL if force else signal.SIGTERM
            if self.tracker.signal_process_group(pgid, sig):
                if not force:
                    # Wait for graceful shutdown
                    start_time = time.time()
                    while time.time() - start_time < timeout:
                        if not self._is_process_group_alive(pgid):
                            logger.info(f"Session {session_id} terminated gracefully")
                            break
                        time.sleep(0.5)
                    else:
                        # Timeout, force kill
                        logger.warning(
                            f"Session {session_id} did not terminate, forcing"
                        )
                        self.tracker.signal_process_group(pgid, signal.SIGKILL)

        # Update session state if tracked
        if session_id in self.sessions:
            self.sessions[session_id].update_state(SessionState.TERMINATED)
            self.unregister_session(session_id)

        # Archive session files
        self.tracker.cleanup_session_files(session_id)

        return True

    def _is_process_group_alive(self, pgid: int) -> bool:
        """Check if any process in the group is alive."""
        try:
            # Send signal 0 to check if process group exists
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except Exception:
            return False

    def cleanup_orphaned_sessions(self) -> List[Dict[str, Any]]:
        """Find and clean up orphaned sessions from stale MCP instances.

        Returns:
            List of cleaned up session information
        """
        logger.info("Searching for orphaned sessions")
        cleaned_up = []

        orphaned_dirs = self.tracker.find_orphaned_sessions()

        for session_dir_str in orphaned_dirs:
            session_dir = Path(session_dir_str)

            # Try to load session state
            session = LaunchSession.load_from_dir(session_dir)

            if session:
                session_info = {
                    "session_id": session.session_id,
                    "launch_file": session.launch_file,
                    "state": session.state.value,
                    "pgid": session.pgid,
                }

                # Try to terminate if still running
                if session.pgid:
                    logger.info(
                        f"Terminating orphaned session {session.session_id} (PGID: {session.pgid})"
                    )
                    self.tracker.signal_process_group(session.pgid, signal.SIGTERM)
                    time.sleep(2)

                    # Force kill if still alive
                    if self._is_process_group_alive(session.pgid):
                        self.tracker.signal_process_group(session.pgid, signal.SIGKILL)

                cleaned_up.append(session_info)

            # Archive the session
            try:
                import shutil

                archive_dir = (
                    self.tracker.base_dir / "archived" / f"orphaned_{session_dir.name}"
                )
                archive_dir.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(session_dir), str(archive_dir))
            except Exception as e:
                logger.error(f"Failed to archive orphaned session {session_dir}: {e}")

        # Clean up stale instance directories
        for instance_dir in self.tracker.base_dir.glob("instance_*"):
            if instance_dir == self.tracker.instance_dir:
                continue

            pid_file = instance_dir / "mcp_server.pid"
            if pid_file.exists():
                try:
                    mcp_pid = int(pid_file.read_text().strip())
                    if not self.tracker.is_process_alive(mcp_pid):
                        logger.info(f"Cleaning up stale instance: {instance_dir}")
                        self.tracker.cleanup_stale_instance(instance_dir)
                except (ValueError, OSError):
                    pass

        logger.info(f"Cleaned up {len(cleaned_up)} orphaned sessions")
        return cleaned_up

    def recover_sessions(self) -> List[LaunchSession]:
        """Recover sessions from the current instance directory.

        Returns:
            List of recovered sessions
        """
        recovered = []
        sessions_dir = self.tracker.instance_dir / "sessions"

        if not sessions_dir.exists():
            return recovered

        for session_dir in sessions_dir.iterdir():
            if not session_dir.is_dir():
                continue

            session = LaunchSession.load_from_dir(session_dir)
            if session:
                # Check if processes are still alive
                if session.pgid and self._is_process_group_alive(session.pgid):
                    logger.info(f"Recovered active session: {session.session_id}")
                    self.register_session(session)
                    recovered.append(session)
                else:
                    # Session is dead, mark as terminated
                    logger.info(f"Found dead session: {session.session_id}")
                    session.update_state(
                        SessionState.TERMINATED, "Process no longer running"
                    )
                    self.tracker.cleanup_session_files(session.session_id)

        return recovered
