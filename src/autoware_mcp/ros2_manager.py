"""ROS2 environment and communication interface."""

import os
import subprocess
import asyncio
from typing import Optional, List, Dict, Any

from .config import get_config
from .logging import get_logger

logger = get_logger("ros2_manager")


class ROS2Manager:
    """Provides interface to ROS2 environment and running nodes/topics."""

    def __init__(self):
        self.config = get_config()
        self._ros_env = None
        self._initialized = False

    def get_ros_env(self) -> Dict[str, str]:
        """Get ROS2 environment variables."""
        if self._ros_env is None:
            self._ros_env = self._setup_ros_env()
        return self._ros_env

    def _setup_ros_env(self) -> Dict[str, str]:
        """Check and use existing ROS2 environment variables.

        This does NOT source setup.bash - it expects the user to have already
        sourced the appropriate ROS2/Autoware environment before running the server.
        """
        env = os.environ.copy()

        # Check if ROS2 environment is already set up
        if "ROS_ROOT" in env or "AMENT_PREFIX_PATH" in env:
            logger.info("Using existing ROS2 environment from user's shell")

            # Log detected ROS settings for debugging
            if "ROS_DISTRO" in env:
                logger.debug(f"ROS_DISTRO: {env['ROS_DISTRO']}")
            if "ROS_DOMAIN_ID" in env:
                logger.debug(f"ROS_DOMAIN_ID: {env['ROS_DOMAIN_ID']}")
        else:
            logger.warning(
                "ROS2 environment not detected. Please source your ROS2/Autoware "
                "setup.bash before starting the MCP server."
            )

        # Only set ROS_DOMAIN_ID if not already set (don't override user's setting)
        if "ROS_DOMAIN_ID" not in env:
            env["ROS_DOMAIN_ID"] = "0"
            logger.debug("ROS_DOMAIN_ID not set, defaulting to 0")

        return env

    async def initialize(self) -> bool:
        """Initialize ROS2 environment."""
        if self._initialized:
            return True

        try:
            # Check if ROS2 is available - try different commands
            commands_to_try = [
                ["ros2", "--version"],
                ["ros2", "daemon", "status"],
                ["ros2", "topic", "list", "--spin-time", "0"],
            ]

            ros2_available = False
            for cmd in commands_to_try:
                try:
                    result = await self.run_command(cmd, timeout=5.0)
                    if result["returncode"] == 0:
                        ros2_available = True
                        if "--version" in cmd:
                            logger.info(f"ROS2 version: {result['stdout'].strip()}")
                        break
                except Exception:
                    continue

            if not ros2_available:
                logger.warning("ROS2 commands not responding, but will continue")

            self._initialized = True
            logger.info("ROS2 interface initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize ROS2 interface: {e}")
            return False

    async def run_command(
        self, cmd: List[str], timeout: Optional[float] = None, check: bool = False
    ) -> Dict[str, Any]:
        """
        Run a ROS2 command asynchronously.

        Args:
            cmd: Command to run as list of strings
            timeout: Optional timeout in seconds
            check: Whether to raise exception on non-zero return code

        Returns:
            Dictionary with returncode, stdout, and stderr
        """
        env = self.get_ros_env()
        timeout = timeout or self.config.server.timeout

        try:
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env=env,
            )

            stdout, stderr = await asyncio.wait_for(
                process.communicate(), timeout=timeout
            )

            result = {
                "returncode": process.returncode,
                "stdout": stdout.decode("utf-8"),
                "stderr": stderr.decode("utf-8"),
            }

            if check and process.returncode != 0:
                raise subprocess.CalledProcessError(
                    process.returncode, cmd, stdout, stderr
                )

            return result

        except asyncio.TimeoutError:
            if process:
                process.kill()
                await process.wait()
            raise TimeoutError(f"Command timed out after {timeout}s: {' '.join(cmd)}")
        except Exception as e:
            logger.error(f"Error running command {' '.join(cmd)}: {e}")
            raise

    async def list_nodes(self) -> List[str]:
        """List all active ROS2 nodes."""
        result = await self.run_command(["ros2", "node", "list"])
        if result["returncode"] == 0:
            return [n.strip() for n in result["stdout"].splitlines() if n.strip()]
        return []

    async def list_topics(self) -> List[str]:
        """List all active ROS2 topics."""
        result = await self.run_command(["ros2", "topic", "list"])
        if result["returncode"] == 0:
            return [t.strip() for t in result["stdout"].splitlines() if t.strip()]
        return []

    async def get_node_info(self, node_name: str) -> Dict[str, Any]:
        """Get information about a specific node."""
        result = await self.run_command(["ros2", "node", "info", node_name])
        if result["returncode"] == 0:
            # Parse node info output
            info = {"name": node_name, "raw": result["stdout"]}
            # Additional parsing can be added here
            return info
        return {}

    async def list_services(self) -> List[str]:
        """List all active ROS2 services."""
        result = await self.run_command(["ros2", "service", "list"])
        if result["returncode"] == 0:
            return [s.strip() for s in result["stdout"].splitlines() if s.strip()]
        return []

    async def get_topic_info(self, topic_name: str) -> Dict[str, Any]:
        """Get information about a specific topic."""
        result = await self.run_command(["ros2", "topic", "info", topic_name])
        if result["returncode"] == 0:
            info = {"name": topic_name, "raw": result["stdout"]}
            # Parse for type, publishers, subscribers
            lines = result["stdout"].splitlines()
            for line in lines:
                if line.strip().startswith("Type:"):
                    info["type"] = line.split(":", 1)[1].strip()
                elif line.strip().startswith("Publisher count:"):
                    info["publisher_count"] = int(line.split(":", 1)[1].strip())
                elif line.strip().startswith("Subscription count:"):
                    info["subscription_count"] = int(line.split(":", 1)[1].strip())
            return info
        return {}

    async def get_topic_hz(
        self, topic_name: str, duration: float = 5.0
    ) -> Dict[str, Any]:
        """Get publishing frequency of a topic."""
        try:
            result = await self.run_command(
                ["ros2", "topic", "hz", topic_name, "--window", str(int(duration))],
                timeout=duration + 2.0,
            )
            if result["returncode"] == 0:
                # Parse hz output
                lines = result["stdout"].splitlines()
                hz_info = {"topic": topic_name, "raw": result["stdout"]}
                for line in lines:
                    if "average rate:" in line.lower():
                        try:
                            hz_value = float(line.split(":")[-1].strip().split()[0])
                            hz_info["average_hz"] = hz_value
                        except (ValueError, IndexError):
                            pass
                return hz_info
        except TimeoutError:
            logger.warning(f"Topic hz check timed out for {topic_name}")
        except Exception as e:
            logger.error(f"Error getting topic hz for {topic_name}: {e}")

        return {"topic": topic_name, "error": "Unable to measure frequency"}

    async def echo_topic(self, topic_name: str, count: int = 1) -> Dict[str, Any]:
        """Echo messages from a topic."""
        try:
            result = await self.run_command(
                [
                    "ros2",
                    "topic",
                    "echo",
                    topic_name,
                    "--once" if count == 1 else f"--times {count}",
                ],
                timeout=10.0,
            )
            if result["returncode"] == 0:
                return {
                    "topic": topic_name,
                    "messages": result["stdout"],
                    "count": count,
                }
        except Exception as e:
            logger.error(f"Error echoing topic {topic_name}: {e}")

        return {"topic": topic_name, "error": "Unable to echo topic"}

    async def call_service(
        self, service_name: str, service_type: str, request_data: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Call a ROS2 service with the given request data.

        Args:
            service_name: Name of the service to call
            service_type: Type of the service (e.g., "std_srvs/srv/Trigger")
            request_data: Request data as a dictionary

        Returns:
            Dictionary with success, data, and optional error fields
        """
        try:
            import json

            # Build command
            cmd = ["ros2", "service", "call", service_name, service_type]

            # Add request data if provided
            if request_data:
                # Convert dict to JSON string for ROS2 CLI
                request_str = json.dumps(request_data)
                cmd.append(request_str)
            else:
                cmd.append("{}")

            # Execute service call
            result = await self.run_command(cmd, timeout=10.0)

            if result["returncode"] != 0:
                logger.error(f"Service call failed: {result['stderr']}")
                return {
                    "success": False,
                    "error": result["stderr"] or "Service call failed",
                }

            # Parse the response
            stdout = result["stdout"]

            # Check for common success patterns in output
            if "success: true" in stdout.lower() or "successfully" in stdout.lower():
                return {"success": True, "data": {"raw_output": stdout}}
            elif "success: false" in stdout.lower() or "failed" in stdout.lower():
                return {
                    "success": False,
                    "data": {"raw_output": stdout},
                    "error": "Service returned failure",
                }
            else:
                # Assume success if call completed without error
                return {"success": True, "data": {"raw_output": stdout}}

        except Exception as e:
            logger.error(f"Error calling service {service_name}: {e}")
            return {"success": False, "error": str(e)}

    async def publish_to_topic(
        self, topic_name: str, message_type: str, message_data: Dict[str, Any]
    ) -> bool:
        """
        Publish a message to a ROS2 topic.

        Args:
            topic_name: Name of the topic to publish to
            message_type: Type of the message (e.g., "geometry_msgs/msg/Twist")
            message_data: Message data as a dictionary

        Returns:
            True if publish succeeded, False otherwise
        """
        try:
            import json

            # Convert message data to JSON string
            message_str = json.dumps(message_data)

            # Build command
            cmd = [
                "ros2",
                "topic",
                "pub",
                "--once",  # Publish once and exit
                topic_name,
                message_type,
                message_str,
            ]

            # Execute publish command
            result = await self.run_command(cmd, timeout=5.0)

            if result["returncode"] != 0:
                logger.error(f"Topic publish failed: {result['stderr']}")
                return False

            return True

        except Exception as e:
            logger.error(f"Error publishing to topic {topic_name}: {e}")
            return False

    async def check_autoware_nodes(self) -> Dict[str, Any]:
        """Check for Autoware-specific nodes and their status."""
        nodes = await self.list_nodes()
        topics = await self.list_topics()

        # Categorize nodes by Autoware components
        autoware_nodes = {
            "perception": [],
            "planning": [],
            "control": [],
            "localization": [],
            "map": [],
            "system": [],
            "other": [],
        }

        for node in nodes:
            node_lower = node.lower()
            if (
                "perception" in node_lower
                or "lidar" in node_lower
                or "camera" in node_lower
            ):
                autoware_nodes["perception"].append(node)
            elif (
                "planning" in node_lower
                or "behavior" in node_lower
                or "motion" in node_lower
            ):
                autoware_nodes["planning"].append(node)
            elif "control" in node_lower or "vehicle" in node_lower:
                autoware_nodes["control"].append(node)
            elif (
                "localization" in node_lower
                or "pose" in node_lower
                or "ekf" in node_lower
            ):
                autoware_nodes["localization"].append(node)
            elif "map" in node_lower:
                autoware_nodes["map"].append(node)
            elif "system" in node_lower or "diagnostic" in node_lower:
                autoware_nodes["system"].append(node)
            else:
                autoware_nodes["other"].append(node)

        # Check for common Autoware topics
        autoware_topics = []
        topic_patterns = [
            "/tf",
            "/map",
            "/pointcloud",
            "/image",
            "/twist",
            "/trajectory",
            "/control",
            "/planning",
            "/perception",
            "/localization",
            "/diagnostics",
        ]

        for topic in topics:
            for pattern in topic_patterns:
                if pattern in topic.lower():
                    autoware_topics.append(topic)
                    break

        return {
            "total_nodes": len(nodes),
            "total_topics": len(topics),
            "autoware_nodes": autoware_nodes,
            "autoware_topics": autoware_topics,
            "autoware_running": len(autoware_topics) > 0,
        }

    async def cleanup(self):
        """Clean up ROS2 interface resources."""
        if self._initialized:
            self._initialized = False
            logger.info("ROS2 interface cleaned up")


# Global ROS2 manager instance
_ros2_manager: Optional[ROS2Manager] = None


def get_ros2_manager() -> ROS2Manager:
    """Get the global ROS2 manager instance."""
    global _ros2_manager
    if _ros2_manager is None:
        _ros2_manager = ROS2Manager()
    return _ros2_manager
