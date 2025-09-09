# Quick Start

## Prerequisites

1. **Autoware installed** with environment sourced
2. **Python 3.10+** with uv package manager
3. **Claude Desktop** or MCP-compatible client

## Installation

```bash
# Clone repository
git clone https://github.com/autoware/autoware-mcp
cd autoware-mcp

# Install dependencies
uv sync --all-extras --dev

# Source Autoware environment
source ~/autoware/install/setup.bash
```

## Basic Usage

### 1. Start Planning Simulation

```python
# Start Autoware planning simulation
result = await start_launch("planning_simulator.launch.xml", {
    "map_path": "~/autoware_map/sample-map-planning",
    "vehicle_model": "sample_vehicle",
    "sensor_model": "sample_sensor_kit"
})
session_id = result["session_id"]
```

### 2. Initialize and Navigate

```python
# Set initial pose
await initialize_localization({
    "position": {"x": 3752.34, "y": 73736.09, "z": 0},
    "orientation": {"x": 0, "y": 0, "z": -0.958, "w": 0.285}
})

# Set goal
await set_route({
    "position": {"x": 3729.46, "y": 73727.25, "z": 0},
    "orientation": {"x": 0, "y": 0, "z": -0.025, "w": 0.999}
})

# Start autonomous driving
await set_operation_mode("autonomous")
```

### 3. Monitor Progress

```python
# Monitor until arrival
while True:
    state = await get_vehicle_state()
    route = await get_current_route()
    
    print(f"Speed: {state['status']['speed']:.1f} m/s")
    print(f"Route state: {route['state']}")
    
    if route["state"] == "ARRIVED":
        break
    
    await asyncio.sleep(1)
```

### 4. Clean Up

```python
# Stop autonomous mode
await set_operation_mode("stop")

# Stop simulation
await stop_launch(session_id)
```

## Common Commands

### Health Check
```python
health = await health_check()
print(f"Status: {health['status']}")
print(f"ROS2: {health['ros2']['status']}")
```

### List Active Nodes
```python
nodes = await list_ros2_nodes()
print(f"Active nodes: {len(nodes)}")
```

### Capture Perception
```python
# Capture camera view
camera = await capture_camera_view("front")
print(f"Image saved: {camera['image_path']}")

# Get detected objects
objects = await get_detected_objects("vehicle")
print(f"Vehicles detected: {len(objects['objects'])}")
```

## Troubleshooting

### No ROS2 nodes found
```bash
# Ensure ROS2 environment is sourced
source ~/autoware/install/setup.bash
export ROS_DOMAIN_ID=1
```

### Launch fails to start
```python
# Check logs
logs = await get_session_logs(session_id, lines=50)
print(logs["content"])
```

### MCP connection lost
```bash
# In Claude Desktop, reconnect with:
/mcp
```