# Quick Start

Get your first autonomous driving mission running in 5 minutes!

## Prerequisites

Before starting, ensure you have:
- ✅ [Installed Autoware MCP](./installation.md)
- ✅ ROS2 environment configured
- ✅ (Optional) Autoware simulation ready

## Step 1: Start the Simulation

First, launch the Autoware planning simulation:

```bash
# Navigate to the project directory
cd ~/autoware-mcp

# Start the planning simulation
./scripts/run_planning_simulation.sh start

# Wait for initialization (about 20 seconds)
# You should see: "Planning simulation is ready!"
```

## Step 2: Start the MCP Server

In a new terminal:

```bash
# Navigate to project directory
cd ~/autoware-mcp

# Start the MCP server
uv run autoware-mcp

# You should see the FastMCP banner and "Server ready"
```

## Step 3: Run Your First Mission

### Option A: Using Python Script

```bash
# Run the example autonomous driving script
uv run python examples/autonomous_drive_mcp_tools.py

# The vehicle will:
# 1. Initialize at the starting position
# 2. Plan a route to the goal
# 3. Drive autonomously to the destination
# 4. Report progress in real-time
```

### Option B: Using Claude Desktop

1. Open Claude Desktop
2. Start a new conversation
3. Say: "Connect to the Autoware MCP server and drive the vehicle to the goal position"

### Option C: Manual Control via CLI

```python
# Start Python with MCP client
uv run python

>>> from examples.autonomous_drive_mcp_tools import *
>>> import asyncio

>>> # Initialize and run mission
>>> asyncio.run(main())
```

## Step 4: Monitor Progress

Watch the vehicle's progress in real-time:

```
Mission Progress:
Position: x=3752.34, y=73736.09
Speed: 2.45 m/s
Distance to goal: 35.2 m
Progress: ████████░░ 78%
Status: Driving autonomously...
```

## What Just Happened?

You successfully:
1. 🚀 Launched an Autoware simulation environment
2. 🔌 Started the MCP server to bridge AI and Autoware
3. 🚗 Executed an autonomous driving mission
4. 📊 Monitored real-time vehicle telemetry

## Common Operations

### Check System Health

```python
# In Python
import asyncio
from autoware_mcp.client import AutowareMCPClient

async def check_health():
    client = AutowareMCPClient()
    health = await client.health_check()
    print(f"System healthy: {health['healthy']}")
    print(f"Active nodes: {health['ros2_nodes_count']}")
    
asyncio.run(check_health())
```

### Set a Custom Goal

```python
async def drive_to_position(x, y):
    client = AutowareMCPClient()
    
    # Set initial position
    await client.initialize_localization({
        "position": {"x": 3752.34, "y": 73736.09, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": -0.958, "w": 0.285}
    })
    
    # Set goal
    await client.set_route({
        "goal_pose": {
            "position": {"x": x, "y": y, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
        }
    })
    
    # Start autonomous driving
    await client.set_operation_mode("autonomous")
```

### Monitor Vehicle State

```python
async def monitor_vehicle():
    client = AutowareMCPClient()
    
    while True:
        state = await client.get_vehicle_state()
        print(f"Speed: {state['status']['velocity']:.2f} m/s")
        print(f"Position: ({state['kinematics']['pose']['position']['x']:.2f}, "
              f"{state['kinematics']['pose']['position']['y']:.2f})")
        await asyncio.sleep(1)
```

## Quick Examples

### 1. Emergency Stop

```python
# Immediate stop
await client.set_operation_mode("stop")
```

### 2. Launch Management

```python
# Start a ROS2 launch file
result = await client.start_launch("my_nodes.launch.py")
session_id = result["session_id"]

# Check status
status = await client.get_session_status(session_id)
print(f"Launch state: {status['state']}")

# Stop when done
await client.stop_launch(session_id)
```

### 3. Custom Waypoints

```python
# Drive through multiple waypoints
waypoints = [
    {"x": 3760, "y": 73740, "z": 0},
    {"x": 3770, "y": 73750, "z": 0},
    {"x": 3780, "y": 73760, "z": 0}
]

await client.set_route_points(waypoints)
```

## Troubleshooting Quick Start

### Simulation Won't Start

```bash
# Check if already running
./scripts/run_planning_simulation.sh status

# Force restart
./scripts/run_planning_simulation.sh stop
./scripts/run_planning_simulation.sh start
```

### MCP Server Connection Issues

```bash
# Check if server is running
ps aux | grep autoware-mcp

# Check ROS2 communication
ros2 topic list

# Verify environment
echo $ROS_DOMAIN_ID  # Should match your configuration
```

### Vehicle Not Moving

```python
# Check operation mode
state = await client.monitor_operation_mode()
print(f"Current mode: {state['mode']}")  # Should be "autonomous"

# Check route status  
route = await client.get_current_route()
print(f"Route state: {route['state']}")  # Should be "SET" (2)

# Check for obstacles
diagnostics = await client.monitor_diagnostics()
print(f"Obstacles detected: {diagnostics.get('obstacles', [])}")
```

## What's Next?

Now that you've completed your first mission:

1. **Learn More**: Read about [Core Concepts](../user-guide/core-concepts.md)
2. **Explore Examples**: Try [Advanced Driving Scenarios](../user-guide/autonomous-driving.md)
3. **Customize**: Create [Custom Launch Files](../advanced/custom-launch.md)
4. **Develop**: Check the [API Reference](../developer/api-reference.md)

## Tips for Success

- 🔄 Always initialize localization before setting routes
- ⏱️ Wait 5 seconds after localization for stability
- 🗺️ Ensure your goal position is reachable on the map
- 🛑 Always have an emergency stop ready
- 📊 Monitor diagnostics for early issue detection

Congratulations on completing your first autonomous driving mission! 🎉