"""Launch session representation and state management."""

import uuid
import json
import logging
import subprocess
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field, asdict

logger = logging.getLogger(__name__)


class SessionState(Enum):
    """Launch session states."""

    INITIALIZED = "initialized"
    STARTING = "starting"
    RUNNING = "running"
    PAUSED = "paused"
    RESUMING = "resuming"
    STOPPING = "stopping"
    TERMINATED = "terminated"
    ERROR = "error"


@dataclass
class NodeInfo:
    """Information about a single ROS2 node."""

    name: str
    pid: int
    started_at: str
    restart_count: int = 0
    critical: bool = False
    status: str = "unknown"


@dataclass
class LaunchSession:
    """Represents a launch session with process tracking."""

    session_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    launch_file: str = ""
    state: SessionState = SessionState.INITIALIZED
    main_pid: Optional[int] = None
    pgid: Optional[int] = None
    nodes: List[NodeInfo] = field(default_factory=list)
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now().isoformat())
    parameters: Dict[str, Any] = field(default_factory=dict)
    error_message: Optional[str] = None
    logs_path: Optional[str] = None

    def __post_init__(self):
        """Initialize session directory structure."""
        self.session_dir = self._create_session_directory()

    def _create_session_directory(self) -> Path:
        """Create directory structure for this session."""
        from .process_tracker import ProcessTracker

        tracker = ProcessTracker()
        session_dir = tracker.get_session_dir(self.session_id)
        session_dir.mkdir(parents=True, exist_ok=True)
        return session_dir

    def update_state(
        self, new_state: SessionState, error_message: Optional[str] = None
    ):
        """Update session state and persist."""
        self.state = new_state
        self.updated_at = datetime.now().isoformat()
        if error_message:
            self.error_message = error_message
        self.save_state()

    def save_state(self):
        """Persist session state to disk."""
        # Check if session directory still exists (might have been archived)
        if not self.session_dir.exists():
            return

        state_file = self.session_dir / "state.json"
        state_data = {
            "session_id": self.session_id,
            "launch_file": self.launch_file,
            "state": self.state.value,
            "main_pid": self.main_pid,
            "pgid": self.pgid,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "parameters": self.parameters,
            "error_message": self.error_message,
            "logs_path": str(self.logs_path) if self.logs_path else None,
            "nodes": [asdict(node) for node in self.nodes],
        }

        with open(state_file, "w") as f:
            json.dump(state_data, f, indent=2)

    @classmethod
    def load_from_dir(cls, session_dir: Path) -> Optional["LaunchSession"]:
        """Load session from directory."""
        state_file = session_dir / "state.json"
        if not state_file.exists():
            return None

        try:
            with open(state_file) as f:
                data = json.load(f)

            session = cls(
                session_id=data["session_id"],
                launch_file=data["launch_file"],
                state=SessionState(data["state"]),
                main_pid=data.get("main_pid"),
                pgid=data.get("pgid"),
                created_at=data["created_at"],
                updated_at=data["updated_at"],
                parameters=data.get("parameters", {}),
                error_message=data.get("error_message"),
                logs_path=data.get("logs_path"),
            )

            # Load nodes
            for node_data in data.get("nodes", []):
                session.nodes.append(NodeInfo(**node_data))

            return session

        except Exception as e:
            logger.error(f"Failed to load session from {session_dir}: {e}")
            return None

    def is_active(self) -> bool:
        """Check if session is in an active state."""
        return self.state in [
            SessionState.STARTING,
            SessionState.RUNNING,
            SessionState.PAUSED,
            SessionState.RESUMING,
        ]

    def get_status_dict(self) -> Dict[str, Any]:
        """Get session status as dictionary."""
        return {
            "session_id": self.session_id,
            "launch_file": self.launch_file,
            "state": self.state.value,
            "main_pid": self.main_pid,
            "pgid": self.pgid,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "node_count": len(self.nodes),
            "nodes": [asdict(node) for node in self.nodes],
            "error_message": self.error_message,
            "logs_path": str(self.logs_path) if self.logs_path else None,
            "is_active": self.is_active(),
        }
