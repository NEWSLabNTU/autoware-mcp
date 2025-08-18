"""Unit tests for configuration management."""

import unittest
import tempfile
from pathlib import Path
import yaml
import os

from autoware_mcp.config import MCPConfig, AutowareConfig, ServerConfig


class TestAutowareConfig(unittest.TestCase):
    """Test Autoware configuration."""
    
    def test_default_config(self):
        """Test default configuration values."""
        config = AutowareConfig()
        self.assertEqual(config.ros_distro, "humble")
        self.assertIsInstance(config.workspace_path, Path)
    
    def test_workspace_validation(self):
        """Test workspace path validation."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Valid path
            config = AutowareConfig(workspace_path=Path(tmpdir))
            self.assertEqual(config.workspace_path, Path(tmpdir))
            
            # Invalid path should raise error
            with self.assertRaises(ValueError):
                AutowareConfig(workspace_path=Path("/nonexistent/path"))
    
    def test_setup_bash_auto_detection(self):
        """Test automatic setup.bash detection."""
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace = Path(tmpdir)
            install_dir = workspace / "install"
            install_dir.mkdir()
            setup_bash = install_dir / "setup.bash"
            setup_bash.touch()
            
            config = AutowareConfig(workspace_path=workspace)
            self.assertEqual(config.setup_bash, setup_bash)


class TestServerConfig(unittest.TestCase):
    """Test server configuration."""
    
    def test_default_values(self):
        """Test default server configuration."""
        config = ServerConfig()
        self.assertEqual(config.host, "localhost")
        self.assertEqual(config.port, 8080)
        self.assertEqual(config.log_level, "INFO")
        self.assertEqual(config.max_connections, 10)
        self.assertEqual(config.timeout, 30.0)
    
    def test_custom_values(self):
        """Test custom server configuration."""
        config = ServerConfig(
            host="0.0.0.0",
            port=9090,
            log_level="DEBUG",
            max_connections=20,
            timeout=60.0
        )
        self.assertEqual(config.host, "0.0.0.0")
        self.assertEqual(config.port, 9090)
        self.assertEqual(config.log_level, "DEBUG")
        self.assertEqual(config.max_connections, 20)
        self.assertEqual(config.timeout, 60.0)


class TestMCPConfig(unittest.TestCase):
    """Test complete MCP configuration."""
    
    def test_from_file(self):
        """Test loading configuration from file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create workspace for validation
            workspace = Path(tmpdir) / "workspace"
            workspace.mkdir()
            
            # Create config file
            config_file = Path(tmpdir) / "config.yaml"
            config_data = {
                "autoware": {
                    "workspace_path": str(workspace),
                    "ros_distro": "iron"
                },
                "server": {
                    "port": 9000,
                    "log_level": "DEBUG"
                }
            }
            
            with open(config_file, "w") as f:
                yaml.dump(config_data, f)
            
            # Load config
            config = MCPConfig.from_file(config_file)
            self.assertEqual(config.autoware.workspace_path, workspace)
            self.assertEqual(config.autoware.ros_distro, "iron")
            self.assertEqual(config.server.port, 9000)
            self.assertEqual(config.server.log_level, "DEBUG")
    
    def test_from_env(self):
        """Test loading configuration from environment variables."""
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace = Path(tmpdir)
            
            # Set environment variables
            env_backup = os.environ.copy()
            try:
                os.environ["AUTOWARE_WORKSPACE"] = str(workspace)
                os.environ["ROS_DISTRO"] = "rolling"
                os.environ["MCP_LOG_LEVEL"] = "WARNING"
                
                config = MCPConfig.from_env()
                self.assertEqual(config.autoware.workspace_path, workspace)
                self.assertEqual(config.autoware.ros_distro, "rolling")
                self.assertEqual(config.server.log_level, "WARNING")
                
            finally:
                # Restore environment
                os.environ.clear()
                os.environ.update(env_backup)
    
    def test_save_config(self):
        """Test saving configuration to file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace = Path(tmpdir) / "workspace"
            workspace.mkdir()
            
            config_file = Path(tmpdir) / "saved_config.yaml"
            
            config = MCPConfig()
            config.autoware.workspace_path = workspace
            config.server.port = 8888
            
            config.save(config_file)
            
            # Verify saved file
            self.assertTrue(config_file.exists())
            
            with open(config_file, "r") as f:
                saved_data = yaml.safe_load(f)
            
            self.assertEqual(saved_data["autoware"]["workspace_path"], str(workspace))
            self.assertEqual(saved_data["server"]["port"], 8888)


if __name__ == "__main__":
    unittest.main()