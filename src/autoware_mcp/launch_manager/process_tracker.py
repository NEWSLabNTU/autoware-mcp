"""Process tracking with PID/PGID file management."""

import os
import signal
import logging
import json
from pathlib import Path
from typing import Optional, List, Dict, Any
import psutil

logger = logging.getLogger(__name__)


class ProcessTracker:
    """Manages PID/PGID files for launch sessions."""

    def __init__(self, base_dir: Optional[Path] = None):
        """Initialize process tracker.

        Args:
            base_dir: Base directory for MCP data (defaults to ./.autoware-mcp)
        """
        self.base_dir = base_dir or Path.cwd() / ".autoware-mcp"
        self.instance_dir = self._get_instance_dir()
        self.instance_dir.mkdir(parents=True, exist_ok=True)
        self._write_instance_info()

    def _get_instance_dir(self) -> Path:
        """Get unique instance directory for this MCP server."""
        # Try to get MCP server port from environment or config
        port = os.environ.get("MCP_SERVER_PORT", "5000")
        pid = os.getpid()
        return self.base_dir / f"instance_{port}_{pid}"

    def _write_instance_info(self):
        """Write MCP server instance information."""
        pid_file = self.instance_dir / "mcp_server.pid"
        port_file = self.instance_dir / "mcp_server.port"

        pid_file.write_text(str(os.getpid()))
        port = os.environ.get("MCP_SERVER_PORT", "5000")
        port_file.write_text(port)

    def get_session_dir(self, session_id: str) -> Path:
        """Get directory for a specific session."""
        return self.instance_dir / "sessions" / session_id

    def write_pid_file(self, session_id: str, pid: int):
        """Write PID file for session."""
        session_dir = self.get_session_dir(session_id)
        session_dir.mkdir(parents=True, exist_ok=True)

        pid_file = session_dir / "session.pid"
        # Atomic write
        tmp_file = pid_file.with_suffix(".tmp")
        tmp_file.write_text(str(pid))
        tmp_file.replace(pid_file)
        logger.debug(f"Wrote PID {pid} for session {session_id}")

    def write_pgid_file(self, session_id: str, pgid: int):
        """Write PGID file for session."""
        session_dir = self.get_session_dir(session_id)
        session_dir.mkdir(parents=True, exist_ok=True)

        pgid_file = session_dir / "session.pgid"
        # Atomic write
        tmp_file = pgid_file.with_suffix(".tmp")
        tmp_file.write_text(str(pgid))
        tmp_file.replace(pgid_file)
        logger.debug(f"Wrote PGID {pgid} for session {session_id}")

    def read_pid(self, session_id: str) -> Optional[int]:
        """Read PID from file."""
        pid_file = self.get_session_dir(session_id) / "session.pid"
        if pid_file.exists():
            try:
                return int(pid_file.read_text().strip())
            except (ValueError, OSError) as e:
                logger.error(f"Failed to read PID for {session_id}: {e}")
        return None

    def read_pgid(self, session_id: str) -> Optional[int]:
        """Read PGID from file."""
        pgid_file = self.get_session_dir(session_id) / "session.pgid"
        if pgid_file.exists():
            try:
                return int(pgid_file.read_text().strip())
            except (ValueError, OSError) as e:
                logger.error(f"Failed to read PGID for {session_id}: {e}")
        return None

    def write_nodes_file(self, session_id: str, nodes: List[Dict[str, Any]]):
        """Write nodes information to file."""
        session_dir = self.get_session_dir(session_id)
        nodes_file = session_dir / "nodes.json"

        data = {"nodes": nodes, "last_updated": os.environ.get("TZ", "UTC")}

        tmp_file = nodes_file.with_suffix(".tmp")
        with open(tmp_file, "w") as f:
            json.dump(data, f, indent=2)
        tmp_file.replace(nodes_file)

    def is_process_alive(self, pid: int) -> bool:
        """Check if process is still alive."""
        try:
            process = psutil.Process(pid)
            return process.is_running()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return False

    def get_process_children(self, pid: int) -> List[int]:
        """Get all child processes of a PID."""
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            return [child.pid for child in children]
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return []

    def signal_process_group(self, pgid: int, sig: signal.Signals) -> bool:
        """Send signal to process group.

        Args:
            pgid: Process group ID
            sig: Signal to send

        Returns:
            True if signal was sent successfully
        """
        try:
            os.killpg(pgid, sig)
            logger.info(f"Sent signal {sig} to process group {pgid}")
            return True
        except ProcessLookupError:
            logger.warning(f"Process group {pgid} not found")
            return False
        except PermissionError:
            logger.error(f"Permission denied to signal process group {pgid}")
            return False
        except Exception as e:
            logger.error(f"Failed to signal process group {pgid}: {e}")
            return False

    def cleanup_session_files(self, session_id: str):
        """Clean up session files after termination."""
        session_dir = self.get_session_dir(session_id)
        if session_dir.exists():
            # Archive instead of deleting for debugging
            archive_dir = self.base_dir / "archived" / session_id
            archive_dir.parent.mkdir(parents=True, exist_ok=True)

            try:
                import shutil

                shutil.move(str(session_dir), str(archive_dir))
                logger.info(f"Archived session {session_id} to {archive_dir}")
            except Exception as e:
                logger.error(f"Failed to archive session {session_id}: {e}")

    def find_orphaned_sessions(self) -> List[str]:
        """Find sessions from stale MCP instances."""
        orphaned = []

        if not self.base_dir.exists():
            return orphaned

        # Check all instance directories
        for instance_dir in self.base_dir.glob("instance_*"):
            if instance_dir == self.instance_dir:
                continue  # Skip our own instance

            # Check if MCP server is still running
            pid_file = instance_dir / "mcp_server.pid"
            if pid_file.exists():
                try:
                    mcp_pid = int(pid_file.read_text().strip())
                    if not self.is_process_alive(mcp_pid):
                        # MCP server is dead, sessions are orphaned
                        sessions_dir = instance_dir / "sessions"
                        if sessions_dir.exists():
                            for session_dir in sessions_dir.iterdir():
                                if session_dir.is_dir():
                                    orphaned.append(str(session_dir))
                except (ValueError, OSError) as e:
                    logger.error(f"Failed to check instance {instance_dir}: {e}")

        return orphaned

    def cleanup_stale_instance(self, instance_dir: Path) -> bool:
        """Clean up a stale MCP instance directory."""
        try:
            # First try to terminate any remaining processes
            sessions_dir = instance_dir / "sessions"
            if sessions_dir.exists():
                for session_dir in sessions_dir.iterdir():
                    if session_dir.is_dir():
                        pgid_file = session_dir / "session.pgid"
                        if pgid_file.exists():
                            try:
                                pgid = int(pgid_file.read_text().strip())
                                self.signal_process_group(pgid, signal.SIGTERM)
                            except (ValueError, OSError):
                                pass

            # Archive the stale instance
            import shutil

            archive_dir = self.base_dir / "archived" / f"stale_{instance_dir.name}"
            archive_dir.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(instance_dir), str(archive_dir))

            logger.info(f"Cleaned up stale instance: {instance_dir}")
            return True

        except Exception as e:
            logger.error(f"Failed to clean up stale instance {instance_dir}: {e}")
            return False
