"""Configuration management for Autoware MCP server."""

import os
import logging
from pathlib import Path
from typing import Optional, Dict, Any
import yaml
from pydantic import BaseModel, Field, validator

logger = logging.getLogger(__name__)


class AutowareConfig(BaseModel):
    """Autoware configuration (optional - for reference only).
    
    Note: The MCP server does not source these paths. Users must source
    their ROS2/Autoware environment before starting the server.
    """
    
    workspace_path: Optional[Path] = Field(
        default=None,
        description="Path to Autoware workspace (optional, for reference only)"
    )
    
    ros_distro: str = Field(
        default="humble",
        description="Expected ROS2 distribution name"
    )
    
    @validator("workspace_path")
    def validate_workspace(cls, v):
        # Workspace path is now optional
        if v is not None and not v.exists():
            logger.warning(f"Workspace path does not exist: {v}")
        return v


class ServerConfig(BaseModel):
    """MCP Server configuration."""
    
    host: str = Field(default="localhost", description="Server host")
    port: int = Field(default=8080, description="Server port")
    log_level: str = Field(default="INFO", description="Logging level")
    max_connections: int = Field(default=10, description="Maximum concurrent connections")
    timeout: float = Field(default=30.0, description="Operation timeout in seconds")


class MCPConfig(BaseModel):
    """Complete MCP configuration."""
    
    autoware: AutowareConfig = Field(default_factory=AutowareConfig)
    server: ServerConfig = Field(default_factory=ServerConfig)
    
    @classmethod
    def from_file(cls, config_path: Path) -> "MCPConfig":
        """Load configuration from YAML file."""
        if not config_path.exists():
            return cls()
        
        with open(config_path, "r") as f:
            data = yaml.safe_load(f) or {}
        
        return cls(**data)
    
    @classmethod
    def from_env(cls) -> "MCPConfig":
        """Load configuration from environment variables."""
        config = cls()
        
        # Override with environment variables
        if workspace := os.getenv("AUTOWARE_WORKSPACE"):
            config.autoware.workspace_path = Path(workspace)
        
        if ros_distro := os.getenv("ROS_DISTRO"):
            config.autoware.ros_distro = ros_distro
        
        if log_level := os.getenv("MCP_LOG_LEVEL"):
            config.server.log_level = log_level
        
        return config
    
    def save(self, config_path: Path):
        """Save configuration to YAML file."""
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with open(config_path, "w") as f:
            yaml.dump(self.dict(), f, default_flow_style=False)


# Global configuration instance
_config: Optional[MCPConfig] = None


def get_config() -> MCPConfig:
    """Get the global configuration instance."""
    global _config
    if _config is None:
        # Try to load from default location
        default_config_path = Path.home() / ".config" / "autoware-mcp" / "config.yaml"
        if default_config_path.exists():
            _config = MCPConfig.from_file(default_config_path)
        else:
            _config = MCPConfig.from_env()
    return _config


def set_config(config: MCPConfig):
    """Set the global configuration instance."""
    global _config
    _config = config