"""Health check and status monitoring for Autoware MCP."""

import asyncio
import psutil
from datetime import datetime
from typing import Dict, Any, Optional
from enum import Enum

from .config import get_config
from .ros2_manager import get_ros2_manager
from .logging import get_logger

logger = get_logger("health")


class HealthStatus(str, Enum):
    """Health status levels."""

    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    UNKNOWN = "unknown"


class SystemMonitor:
    """Monitor system resources and health."""

    def __init__(self):
        self.config = get_config()
        self.ros2_manager = get_ros2_manager()
        self._start_time = datetime.now()

    async def get_system_info(self) -> Dict[str, Any]:
        """Get system information and resource usage."""
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage("/")

            return {
                "cpu": {
                    "percent": cpu_percent,
                    "count": psutil.cpu_count(),
                    "frequency": psutil.cpu_freq().current
                    if psutil.cpu_freq()
                    else None,
                },
                "memory": {
                    "total_gb": round(memory.total / (1024**3), 2),
                    "used_gb": round(memory.used / (1024**3), 2),
                    "available_gb": round(memory.available / (1024**3), 2),
                    "percent": memory.percent,
                },
                "disk": {
                    "total_gb": round(disk.total / (1024**3), 2),
                    "used_gb": round(disk.used / (1024**3), 2),
                    "free_gb": round(disk.free / (1024**3), 2),
                    "percent": disk.percent,
                },
                "network": self._get_network_info(),
                "processes": {
                    "total": len(psutil.pids()),
                    "ros2_processes": self._count_ros2_processes(),
                },
            }
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return {}

    def _get_network_info(self) -> Dict[str, Any]:
        """Get network interface information."""
        try:
            stats = psutil.net_if_stats()
            addrs = psutil.net_if_addrs()

            interfaces = {}
            for iface, stat in stats.items():
                if stat.isup:
                    interfaces[iface] = {
                        "is_up": stat.isup,
                        "speed_mbps": stat.speed,
                        "addresses": [
                            addr.address
                            for addr in addrs.get(iface, [])
                            if addr.family == 2  # IPv4
                        ],
                    }
            return interfaces
        except Exception as e:
            logger.error(f"Error getting network info: {e}")
            return {}

    def _count_ros2_processes(self) -> int:
        """Count ROS2-related processes."""
        count = 0
        try:
            for proc in psutil.process_iter(["name", "cmdline"]):
                try:
                    cmdline = " ".join(proc.info.get("cmdline", []))
                    if "ros2" in cmdline or "/opt/ros" in cmdline:
                        count += 1
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        except Exception as e:
            logger.error(f"Error counting ROS2 processes: {e}")
        return count

    async def check_ros2_health(self) -> Dict[str, Any]:
        """Check ROS2 system health."""
        health = {
            "status": HealthStatus.UNKNOWN,
            "daemon_running": False,
            "nodes": [],
            "topics": [],
            "errors": [],
        }

        try:
            # Check if ROS2 daemon is running
            result = await self.ros2_manager.run_command(
                ["ros2", "daemon", "status"], timeout=5.0
            )
            health["daemon_running"] = "running" in result["stdout"].lower()

            # Get active nodes and topics
            health["nodes"] = await self.ros2_manager.list_nodes()
            health["topics"] = await self.ros2_manager.list_topics()

            # Determine health status
            if health["daemon_running"]:
                if len(health["nodes"]) > 0:
                    health["status"] = HealthStatus.HEALTHY
                else:
                    health["status"] = HealthStatus.DEGRADED
            else:
                health["status"] = HealthStatus.UNHEALTHY
                health["errors"].append("ROS2 daemon is not running")

        except Exception as e:
            health["status"] = HealthStatus.UNHEALTHY
            health["errors"].append(str(e))
            logger.error(f"Error checking ROS2 health: {e}")

        return health

    async def check_autoware_health(self) -> Dict[str, Any]:
        """Check Autoware runtime status by analyzing active nodes and topics."""
        health = {
            "status": HealthStatus.UNKNOWN,
            "workspace_exists": False,
            "setup_bash_exists": False,
            "running": False,
            "components": {},
            "errors": [],
        }

        try:
            # Workspace path is optional and not used for operation
            if self.config.autoware.workspace_path:
                health["workspace_exists"] = (
                    self.config.autoware.workspace_path.exists()
                )
            else:
                health["workspace_exists"] = None  # Not configured

            # Check for running Autoware components by analyzing ROS2 graph
            autoware_status = await self.ros2_manager.check_autoware_nodes()
            health.update(autoware_status)

            # Determine health status based on running components
            if autoware_status["autoware_running"]:
                # Count active component types
                active_components = sum(
                    1 for nodes in autoware_status["autoware_nodes"].values() if nodes
                )

                if active_components >= 3:  # At least 3 component types active
                    health["status"] = HealthStatus.HEALTHY
                elif active_components >= 1:  # Some components active
                    health["status"] = HealthStatus.DEGRADED
                else:
                    health["status"] = HealthStatus.DEGRADED
                    health["errors"].append(
                        "Autoware topics found but no nodes categorized"
                    )
            else:
                health["status"] = HealthStatus.UNHEALTHY
                health["errors"].append("No Autoware components detected")

        except Exception as e:
            health["status"] = HealthStatus.UNHEALTHY
            health["errors"].append(str(e))
            logger.error(f"Error checking Autoware health: {e}")

        return health

    async def get_complete_health_status(self) -> Dict[str, Any]:
        """Get complete health status of the system."""
        uptime = (datetime.now() - self._start_time).total_seconds()

        # Run health checks in parallel
        system_task = asyncio.create_task(self.get_system_info())
        ros2_task = asyncio.create_task(self.check_ros2_health())
        autoware_task = asyncio.create_task(self.check_autoware_health())

        system_info = await system_task
        ros2_health = await ros2_task
        autoware_health = await autoware_task

        # Determine overall status
        statuses = [ros2_health["status"], autoware_health["status"]]
        if HealthStatus.UNHEALTHY in statuses:
            overall_status = HealthStatus.UNHEALTHY
        elif HealthStatus.DEGRADED in statuses:
            overall_status = HealthStatus.DEGRADED
        elif HealthStatus.UNKNOWN in statuses:
            overall_status = HealthStatus.UNKNOWN
        else:
            overall_status = HealthStatus.HEALTHY

        return {
            "timestamp": datetime.now().isoformat(),
            "uptime_seconds": uptime,
            "overall_status": overall_status,
            "system": system_info,
            "ros2": ros2_health,
            "autoware": autoware_health,
            "config": {
                "workspace": str(self.config.autoware.workspace_path),
                "ros_distro": self.config.autoware.ros_distro,
                "log_level": self.config.server.log_level,
            },
        }


# Global system monitor instance
_monitor: Optional[SystemMonitor] = None


def get_monitor() -> SystemMonitor:
    """Get the global system monitor instance."""
    global _monitor
    if _monitor is None:
        _monitor = SystemMonitor()
    return _monitor
