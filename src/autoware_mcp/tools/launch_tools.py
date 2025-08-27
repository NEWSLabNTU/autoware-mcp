"""MCP tools for launch session management."""

from typing import Dict, List, Optional, Any
from ..launch_manager import LaunchSessionManager, LaunchGenerator


class LaunchTools:
    """MCP tools for managing ROS2 launch sessions."""

    def __init__(self, ros2_manager=None):
        """Initialize launch tools.

        Args:
            ros2_manager: Optional ROS2Manager instance
        """
        self.session_manager = LaunchSessionManager(ros2_manager)
        self.generator = LaunchGenerator()

    async def start_launch(
        self,
        launch_file: str,
        parameters: Optional[Dict[str, Any]] = None,
        launch_args: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Start a ROS2 launch file and track the session.

        Args:
            launch_file: Path to the launch file
            parameters: Optional parameters to pass to launch
            launch_args: Optional command line arguments

        Returns:
            Session information including ID and status
        """
        return await self.session_manager.start_launch(
            launch_file, parameters, launch_args
        )

    async def stop_launch(
        self, session_id: str, force: bool = False, timeout: float = 10.0
    ) -> Dict[str, Any]:
        """Stop a launch session gracefully or forcefully.

        Args:
            session_id: Session ID to stop
            force: If True, use SIGKILL immediately
            timeout: Timeout for graceful shutdown (seconds)

        Returns:
            Status of the stop operation
        """
        return await self.session_manager.stop_launch(session_id, force, timeout)

    async def pause_launch(self, session_id: str) -> Dict[str, Any]:
        """Pause a running launch session using SIGSTOP.

        Args:
            session_id: Session ID to pause

        Returns:
            Status of the pause operation
        """
        return await self.session_manager.pause_launch(session_id)

    async def resume_launch(self, session_id: str) -> Dict[str, Any]:
        """Resume a paused launch session using SIGCONT.

        Args:
            session_id: Session ID to resume

        Returns:
            Status of the resume operation
        """
        return await self.session_manager.resume_launch(session_id)

    async def restart_launch(self, session_id: str) -> Dict[str, Any]:
        """Restart a launch session (stop and start with same config).

        Args:
            session_id: Session ID to restart

        Returns:
            Status with old and new session IDs
        """
        return await self.session_manager.restart_launch(session_id)

    async def list_launch_sessions(self) -> List[Dict[str, Any]]:
        """List all launch sessions with their current status.

        Returns:
            List of session information dictionaries
        """
        return await self.session_manager.list_sessions()

    async def get_session_status(self, session_id: str) -> Dict[str, Any]:
        """Get detailed status of a specific launch session.

        Args:
            session_id: Session ID to query

        Returns:
            Detailed session status including PID/PGID, state, nodes
        """
        return await self.session_manager.get_session_status(session_id)

    async def get_session_logs(
        self, session_id: str, lines: int = 100, stream: str = "stdout"
    ) -> Dict[str, Any]:
        """Get buffered logs from a launch session.

        Args:
            session_id: Session ID
            lines: Number of lines to retrieve (default: 100)
            stream: Log stream to read (stdout or stderr)

        Returns:
            Log content and metadata
        """
        return await self.session_manager.get_session_logs(session_id, lines, stream)

    async def cleanup_orphans(self) -> Dict[str, Any]:
        """Clean up orphaned processes from stale MCP instances.

        Scans for sessions from dead MCP servers and terminates them.

        Returns:
            Information about cleaned up sessions
        """
        return await self.session_manager.cleanup_orphans()

    async def generate_launch_file(
        self,
        name: str,
        components: List[str],
        template: Optional[str] = None,
        includes: Optional[List[str]] = None,
        remappings: Optional[Dict[str, str]] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Generate a ROS2 launch file for AI development.

        Creates launch files in .autoware-mcp/generated/launches/ with versioning.

        Args:
            name: Name for the launch file (without extension)
            components: List of components/nodes to include
            template: Optional template (perception_pipeline, planning_pipeline, control_pipeline)
            includes: Other launch files to include
            remappings: Topic remappings
            parameters: Parameters to set

        Returns:
            Path to generated launch file and metadata
        """
        return self.generator.generate_launch_file(
            name, components, template, includes, remappings, parameters
        )

    async def validate_launch_file(self, file_path: str) -> Dict[str, Any]:
        """Validate launch file syntax and structure.

        Args:
            file_path: Path to launch file to validate

        Returns:
            Validation results with success status and any errors
        """
        return self.generator.validate_launch_file(file_path)

    async def generate_node_config(
        self, node_name: str, parameters: Dict[str, Any], format: str = "yaml"
    ) -> Dict[str, Any]:
        """Generate configuration file for a ROS2 node.

        Creates config files in .autoware-mcp/generated/configs/ with versioning.

        Args:
            node_name: Name of the node
            parameters: Parameters to configure
            format: Config format (yaml or json)

        Returns:
            Path to generated config file
        """
        return self.generator.generate_node_config(node_name, parameters, format)

    async def generate_custom_node(
        self,
        name: str,
        language: str = "python",
        node_type: str = "basic",
        interfaces: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Generate a custom ROS2 node template.

        Creates node files in .autoware-mcp/generated/nodes/ with versioning.

        Args:
            name: Name of the node
            language: Programming language (python or cpp)
            node_type: Type of node (basic, service, action)
            interfaces: List of topics/services to interface with

        Returns:
            Path to generated node file
        """
        return self.generator.generate_custom_node(
            name, language, node_type, interfaces
        )

    async def test_launch_file(
        self, file_path: str, dry_run: bool = True
    ) -> Dict[str, Any]:
        """Test a launch file without fully starting nodes (dry-run).

        Args:
            file_path: Path to launch file to test
            dry_run: If True, only validate without starting

        Returns:
            Test results with any errors or warnings
        """
        # First validate syntax
        validation = self.generator.validate_launch_file(file_path)
        if not validation["success"]:
            return validation

        if dry_run:
            # Just return validation results for dry run
            return {
                "success": True,
                "message": "Launch file validation passed",
                "dry_run": True,
            }
        else:
            # Actually try to start and immediately stop
            result = await self.session_manager.start_launch(file_path)
            if result["success"]:
                # Stop it immediately
                await self.session_manager.stop_launch(result["session_id"])
                return {
                    "success": True,
                    "message": "Launch file test successful",
                    "session_id": result["session_id"],
                }
            else:
                return result

    async def diff_launch_sessions(
        self, session_id_1: str, session_id_2: str
    ) -> Dict[str, Any]:
        """Compare configurations of two launch sessions.

        Args:
            session_id_1: First session ID
            session_id_2: Second session ID

        Returns:
            Differences between the sessions
        """
        status_1 = await self.session_manager.get_session_status(session_id_1)
        status_2 = await self.session_manager.get_session_status(session_id_2)

        if not status_1["success"] or not status_2["success"]:
            return {"success": False, "error": "One or both sessions not found"}

        differences = {
            "launch_file": {
                "session_1": status_1.get("launch_file"),
                "session_2": status_2.get("launch_file"),
                "same": status_1.get("launch_file") == status_2.get("launch_file"),
            },
            "state": {
                "session_1": status_1.get("state"),
                "session_2": status_2.get("state"),
                "same": status_1.get("state") == status_2.get("state"),
            },
            "node_count": {
                "session_1": status_1.get("node_count"),
                "session_2": status_2.get("node_count"),
                "same": status_1.get("node_count") == status_2.get("node_count"),
            },
        }

        return {"success": True, "differences": differences}

    async def get_launch_errors(self, session_id: str) -> Dict[str, Any]:
        """Get error diagnostics from a launch session.

        Args:
            session_id: Session ID to get errors from

        Returns:
            Error information and diagnostics
        """
        # Get session status
        status = await self.session_manager.get_session_status(session_id)
        if not status["success"]:
            return status

        # Get stderr logs for errors
        logs = await self.session_manager.get_session_logs(session_id, 200, "stderr")

        return {
            "success": True,
            "session_id": session_id,
            "state": status.get("state"),
            "error_message": status.get("error_message"),
            "stderr_logs": logs.get("content") if logs["success"] else None,
        }

    async def list_generated_files(self) -> Dict[str, List[str]]:
        """List all AI-generated files in the project.

        Returns:
            Dictionary of generated files by category (launches, nodes, configs)
        """
        return self.generator.list_generated_files()

    async def set_session_parameters(
        self, session_id: str, parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Update runtime parameters for a launch session.

        Note: This requires nodes to support dynamic reconfiguration.

        Args:
            session_id: Session ID to update
            parameters: Parameters to set

        Returns:
            Status of parameter update
        """
        # This would require ROS2 parameter client implementation
        # For now, return not implemented
        return {
            "success": False,
            "error": "Runtime parameter updates not yet implemented",
        }
