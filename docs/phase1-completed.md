# Phase 1: Foundation & Core Infrastructure - COMPLETED

## Summary
Phase 1 of the Autoware MCP server has been successfully implemented, providing a **passive observation and interaction interface** for running Autoware instances through the Model Context Protocol. The server does not manage Autoware processes but focuses on monitoring and interacting with already-running Autoware components.

## Implemented Components

### 1. MCP Server Skeleton (`src/autoware_mcp/server.py`)
- FastMCP-based server implementation
- Tool decorators for MCP endpoints
- Async operation support
- Server lifecycle management (initialize/shutdown)

### 2. ROS2 Communication Interface (`src/autoware_mcp/ros2_manager.py`)
- ROS2 environment initialization and setup
- Node, topic, and service discovery
- Topic information and frequency monitoring
- Message capturing and echoing
- Autoware component categorization and status checking
- Command execution with timeout support

### 3. Configuration System (`src/autoware_mcp/config.py`)
- Pydantic-based configuration models
- Support for YAML configuration files
- Environment variable overrides
- Autoware workspace validation
- Automatic setup.bash detection

### 4. Health Monitoring (`src/autoware_mcp/health.py`)
- System resource monitoring (CPU, memory, disk, network)
- ROS2 health checks and daemon status
- **Runtime Autoware component detection and categorization**
- Overall health status aggregation based on active components
- Passive monitoring approach (no process management)

### 5. Logging Infrastructure (`src/autoware_mcp/logging.py`)
- Colored console output
- File logging support
- Configurable log levels
- Structured logging format

### 6. Unit Tests
- Configuration management tests (`tests/test_config.py`)
- Health monitoring tests (`tests/test_health.py`)
- ROS2 manager tests (`tests/test_ros2_manager.py`)
- Comprehensive mocking for async operations

## Available MCP Tools

The following tools are now available through the MCP interface:

### System Monitoring
1. **health_check** - Complete health status including system, ROS2, and running Autoware components
2. **get_system_status** - Current system resource usage (CPU, memory, disk, network)

### ROS2 Observation
3. **list_ros2_nodes** - List all active ROS2 nodes
4. **list_ros2_topics** - List all active ROS2 topics  
5. **list_ros2_services** - List all active ROS2 services
6. **get_node_info** - Detailed information about a specific node
7. **get_topic_info** - Detailed information about a specific topic
8. **get_topic_frequency** - Measure publishing frequency of a topic
9. **echo_topic_messages** - Capture messages from a topic

### Autoware Status
10. **check_autoware_status** - Analyze running Autoware components by category
11. **verify_autoware_installation** - Verify Autoware installation paths (optional)

### Server Management  
12. **get_configuration** - Current MCP server configuration
13. **initialize_server** - Initialize server components
14. **shutdown_server** - Graceful shutdown

## Project Structure

```
autoware-mcp/
├── src/
│   └── autoware_mcp/
│       ├── __init__.py         # Package initialization
│       ├── __main__.py         # Main entry point
│       ├── cli.py              # CLI utilities (test runner)
│       ├── config.py           # Configuration management
│       ├── health.py           # Health monitoring
│       ├── logging.py          # Logging infrastructure
│       ├── ros2_manager.py     # ROS2 operations
│       └── server.py           # Main MCP server
├── tests/
│   ├── __init__.py
│   ├── test_config.py          # Configuration tests
│   ├── test_health.py          # Health monitoring tests
│   └── test_ros2_manager.py    # ROS2 manager tests
├── docs/
│   ├── roadmap.md              # Development roadmap
│   ├── phase1-completed.md     # This document
│   └── usage-guide.md          # Usage guide
└── pyproject.toml              # Project config with entry points
```

## Running the Server

```bash
# Install the package (first time only)
rye sync

# IMPORTANT: Source your ROS2/Autoware environment first!
source ~/autoware/install/setup.bash
# or
source /opt/ros/humble/setup.bash && source ~/autoware/install/setup.bash

# Run the server using entry point
rye run autoware-mcp

# Or activate virtual environment and run directly
source .venv/bin/activate
autoware-mcp
```

**Note:** The server does NOT source any setup files. You must source your ROS2/Autoware environment before starting the server.

## Running Tests

```bash
# Run tests using entry point
rye run autoware-mcp-test

# Or activate virtual environment and run directly  
source .venv/bin/activate
autoware-mcp-test
```

## Configuration

The server can be configured through:

1. **Environment Variables:**
   - `AUTOWARE_WORKSPACE` - Path to Autoware workspace
   - `ROS_DISTRO` - ROS2 distribution (humble, iron, etc.)
   - `MCP_LOG_LEVEL` - Logging level (DEBUG, INFO, WARNING, ERROR)

2. **Configuration File:**
   Create `~/.config/autoware-mcp/config.yaml`:
   ```yaml
   autoware:
     workspace_path: /path/to/autoware/workspace
     ros_distro: humble
   server:
     host: localhost
     port: 8080
     log_level: INFO
   ```

## Testing Phase 1

To verify Phase 1 functionality:

1. **Start Autoware** (user's responsibility):
   ```bash
   # User launches Autoware with their preferred method, e.g.:
   ros2 launch autoware_launch planning_simulator.launch.xml map_path:=... vehicle_model:=...
   ```

2. **Check Autoware Status:**
   ```python
   # Use MCP client to check running components
   status = await mcp_client.call_tool("check_autoware_status")
   print(f"Autoware running: {status['autoware_running']}")
   print(f"Active components: {status['autoware_nodes']}")
   ```

3. **Monitor System Health:**
   ```python
   health = await mcp_client.call_tool("health_check")
   print(f"Overall status: {health['overall_status']}")
   ```

4. **Interact with Topics:**
   ```python
   # List topics
   topics = await mcp_client.call_tool("list_ros2_topics")
   
   # Get topic info
   info = await mcp_client.call_tool("get_topic_info", {"topic_name": "/tf"})
   
   # Measure frequency
   freq = await mcp_client.call_tool("get_topic_frequency", {"topic_name": "/tf"})
   ```

## Next Steps

Phase 1 provides the foundation for Phase 2, which will add:
- Topic subscription and data streaming
- Service discovery and invocation
- Parameter server operations
- Message type introspection

## Design Principles

### Passive Approach
- **No Process Management**: Server does not launch, stop, or manage Autoware processes
- **Observation Focus**: Monitors and interacts with already-running components
- **User Responsibility**: Users launch Autoware using their preferred methods
- **Runtime Detection**: Health checks analyze the ROS2 graph to detect active components

### Benefits of Passive Design
- **Safety**: No risk of accidentally stopping critical Autoware processes
- **Flexibility**: Works with any Autoware launch configuration
- **Simplicity**: Users maintain full control over their Autoware instances
- **Compatibility**: Works regardless of how Autoware was started

## Known Limitations

- FastMCP integration may need adjustment based on the latest FastMCP API
- ROS2 command execution uses subprocess (rclpy integration could be added in future phases)
- Topic message echo has timeout limitations for high-frequency topics
- No authentication/authorization yet (planned for Phase 8)
- Configuration workspace paths are optional for runtime operation

## Dependencies Installed

- fastmcp>=0.1.0 - MCP framework
- pydantic>=2.0.0 - Data validation
- pyyaml>=6.0 - YAML configuration
- psutil>=5.9.0 - System monitoring
- pytest>=7.0.0 - Testing framework
- pytest-asyncio>=0.21.0 - Async test support
- pytest-cov>=4.0.0 - Test coverage

## Success Criteria Met

✅ MCP server starts and responds to queries (14 tools available)  
✅ **Can detect running Autoware components without managing processes**  
✅ Health check provides comprehensive system and component status  
✅ ROS2 observation tools working (nodes, topics, services, frequency, echo)  
✅ Passive monitoring approach implemented successfully  
✅ Logging infrastructure operational with colored output  
✅ Unit tests created with comprehensive async coverage  
✅ Configuration system working with optional workspace validation