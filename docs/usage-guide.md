# Autoware MCP Usage Guide

## Overview
The Autoware MCP server provides a **passive interface** to interact with running Autoware instances. It does not manage Autoware processes but observes and interacts with components that are already running.

## Workflow

### 1. Start Autoware (Your Responsibility)
Launch Autoware using your preferred method:

```bash
# Example: Planning Simulator
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    vehicle_model:=sample_vehicle

# Example: Logging Simulator  
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    vehicle_model:=sample_vehicle

# Example: Custom launch file
ros2 launch my_autoware_config my_custom.launch.xml
```

### 2. Source ROS2/Autoware Environment
**IMPORTANT:** You must source your ROS2/Autoware environment before starting the MCP server:

```bash
# Source your Autoware workspace
source ~/autoware/install/setup.bash

# Or source ROS2 and then Autoware
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash

# Verify environment is set
echo $ROS_DISTRO  # Should show "humble" or your ROS version
```

### 3. Start MCP Server
```bash
cd /home/aeon/repos/autoware-mcp

# Install package with entry points (first time only)
rye sync

# Run the server (AFTER sourcing ROS2/Autoware environment)
rye run autoware-mcp

# Or activate virtual environment and run directly
source .venv/bin/activate
autoware-mcp
```

### 4. Check Autoware Status
Use MCP tools to verify Autoware is detected:

```python
# Check if Autoware components are running
status = await mcp_client.call_tool("check_autoware_status")
print(f"Autoware detected: {status['autoware_running']}")
print(f"Components found:")
for component, nodes in status['autoware_nodes'].items():
    if nodes:
        print(f"  {component}: {len(nodes)} nodes")
```

### 5. Monitor and Interact
```python
# Monitor system health
health = await mcp_client.call_tool("health_check")

# List active topics
topics = await mcp_client.call_tool("list_ros2_topics")

# Get topic information
tf_info = await mcp_client.call_tool("get_topic_info", {"topic_name": "/tf"})

# Monitor topic frequency
freq = await mcp_client.call_tool("get_topic_frequency", {
    "topic_name": "/localization/kinematic_state",
    "duration": 5.0
})

# Capture topic messages
messages = await mcp_client.call_tool("echo_topic_messages", {
    "topic_name": "/planning/mission_planning/route",
    "count": 1
})
```

## Key Principles

### ✅ What the MCP Server Does
- **Observes** running ROS2 nodes and topics
- **Categorizes** Autoware components automatically
- **Monitors** system resources and health
- **Provides** real-time information about topic frequencies
- **Captures** messages from topics for inspection
- **Reports** on the status of running components

### ❌ What the MCP Server Does NOT Do
- **Launch** Autoware processes
- **Stop** or **restart** Autoware components
- **Manage** process lifecycles
- **Modify** Autoware configurations
- **Control** launch sequences

## Component Detection

The server automatically categorizes detected nodes into:

- **Perception**: Lidar, camera, object detection nodes
- **Planning**: Behavior planning, motion planning nodes  
- **Control**: Vehicle control, trajectory following nodes
- **Localization**: Pose estimation, EKF nodes
- **Map**: Map-related processing nodes
- **System**: Diagnostics, monitoring nodes
- **Other**: Uncategorized but detected nodes

## Safety Benefits

This passive approach provides several safety benefits:

1. **No Accidental Shutdowns**: Cannot accidentally stop critical processes
2. **User Control**: You maintain full control over Autoware lifecycle  
3. **Process Isolation**: MCP server issues don't affect Autoware operation
4. **Flexible Integration**: Works with any launch configuration
5. **Observation Only**: Monitoring doesn't interfere with performance

## Troubleshooting

### "Autoware not detected"
- Verify Autoware is actually running: `ros2 node list`
- Check ROS2 environment: `echo $ROS_DISTRO`
- Ensure topics are being published: `ros2 topic list`

### "No topics found" 
- Wait a moment for Autoware to fully initialize
- Some components may take time to start publishing

### "ROS2 daemon not running"
- The server can still work but may be slower
- Daemon is automatically managed, no action needed

## Example Session

```bash
# Terminal 1: Start Autoware
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=...

# Terminal 2: Start MCP Server (in same sourced environment)
source ~/autoware/install/setup.bash  # REQUIRED!
cd /home/aeon/repos/autoware-mcp
rye run autoware-mcp

# Terminal 3: Use MCP client to interact
# (Your AI assistant or MCP client application)
```

## Environment Requirements

The MCP server **does not** source any setup files itself. You must:

1. **Source your ROS2/Autoware environment** before starting the server
2. The server will use whatever ROS2 environment is active in your shell
3. All ROS2 commands are run using your shell's environment

This design ensures:
- No conflicts with your existing ROS2 setup
- The server uses exactly the same environment you use
- No hidden environment modifications
- Full compatibility with any ROS2/Autoware installation method

## Next Development Phases

Phase 2 will add:
- Parameter server interactions
- Service calling capabilities  
- Real-time data streaming
- Enhanced topic filtering

The passive observation principle will be maintained throughout all phases.