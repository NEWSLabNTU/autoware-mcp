# Source Code Structure

## Project Layout

```
autoware-mcp/
├── src/autoware_mcp/           # Main source code
│   ├── server.py               # FastMCP server implementation
│   ├── ad_api_ros2.py          # Autoware AD API interface
│   ├── ros2_manager.py         # ROS2 operations manager
│   ├── perception_bridge.py   # Perception data bridge
│   ├── launch_manager/        # Launch session management
│   │   ├── session_manager.py # Session lifecycle
│   │   ├── process_tracker.py # PID/PGID tracking
│   │   └── generator.py       # Launch file generation
│   └── tools/                 # MCP tool implementations
│       ├── launch_tools.py    # Launch management tools
│       └── perception_tools.py# Perception tools
├── tests/                     # Comprehensive test suite
├── examples/                  # Usage examples
└── book/                      # Documentation
```

## Key Components

### Server Core (`server.py`)
- FastMCP server setup and tool registration
- Request routing and response handling
- Lifecycle management

### ROS2 Manager (`ros2_manager.py`)
- Node and topic discovery
- Service calls and topic publishing
- Command execution wrapper

### AD API Interface (`ad_api_ros2.py`)
- Autoware-specific operations
- Vehicle control commands
- Route and localization management

### Launch Manager (`launch_manager/`)
- Process group management
- Session state tracking
- Orphan cleanup
- Launch file generation

### Perception Bridge (`perception_bridge.py`)
- Camera image capture
- LiDAR visualization
- Object detection interface

## Runtime Directories

```
.autoware-mcp/
├── instance_<port>_<pid>/     # Per-instance tracking
│   └── sessions/              # Active launch sessions
├── generated/                 # AI-generated files
│   ├── launches/              # Launch files
│   └── configs/               # Node configurations
└── logs/                      # Session logs
    └── session_<id>/
        ├── stdout.log
        └── stderr.log
```