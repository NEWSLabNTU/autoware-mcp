# Core Concepts

Understanding these fundamental concepts will help you effectively use the Autoware MCP Server.

## Model Context Protocol (MCP)

MCP is a standardized protocol that enables AI models to interact with external systems through a well-defined interface.

### Key MCP Components

1. **Tools**: Functions that the AI can call to perform actions
2. **Resources**: Data sources the AI can read from
3. **Context**: Shared state between the AI and the system
4. **Transport**: Communication layer (stdio, HTTP, WebSocket)

### MCP in Autoware Context

```
AI Agent <--[MCP Protocol]--> MCP Server <--[ROS2]--> Autoware
```

The MCP server translates high-level AI commands into ROS2 service calls and topics that Autoware understands.

## Autoware Stack

Autoware is a complete autonomous driving software stack consisting of:

### Perception
- **Sensors**: LiDAR, cameras, radar, IMU
- **Detection**: Object detection and classification
- **Tracking**: Multi-object tracking
- **Prediction**: Trajectory prediction

### Planning
- **Mission Planning**: High-level route planning
- **Behavior Planning**: Lane changes, intersections
- **Motion Planning**: Trajectory generation
- **Velocity Planning**: Speed optimization

### Control
- **Trajectory Following**: Path tracking
- **Vehicle Interface**: Actuator commands
- **Feedback Control**: Closed-loop control

## Key Concepts

### 1. Localization

The process of determining the vehicle's position on the map.

```python
# Initialize localization with a known position
await client.initialize_localization({
    "pose": {
        "position": {"x": 3752.34, "y": 73736.09, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": -0.958, "w": 0.285}
    }
})
```

**Important**: Always initialize localization before attempting to drive!

### 2. Route Planning

Creating a path from the current position to a goal.

```python
# Set a route to a destination
await client.set_route({
    "goal_pose": {
        "position": {"x": 3798.71, "y": 73775.45, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0.862, "w": 0.506}
    }
})
```

**Route States**:
- `UNSET` (1): No route planned
- `SET` (2): Route is ready
- `ARRIVED` (3): Reached destination
- `CHANGING` (4): Route being modified

### 3. Operation Modes

Control how the vehicle behaves:

- **STOP**: Vehicle is stopped, no autonomous control
- **AUTONOMOUS**: Full autonomous driving
- **LOCAL**: Manual control via local interface
- **REMOTE**: Remote control mode

```python
# Enable autonomous driving
await client.set_operation_mode("autonomous")
```

### 4. Vehicle State

Real-time information about the vehicle:

```python
state = await client.get_vehicle_state()

# Returns:
{
    "kinematics": {
        "pose": {"position": {...}, "orientation": {...}},
        "velocity": {"linear": {...}, "angular": {...}}
    },
    "status": {
        "velocity": 2.45,  # m/s
        "steering_angle": 0.1,  # radians
        "gear": "DRIVE"
    }
}
```

### 5. Launch Sessions

Managed ROS2 launch file execution:

```python
# Start a launch session
session = await client.start_launch("my_nodes.launch.py")

# Monitor session
status = await client.get_session_status(session["session_id"])

# Stop when done
await client.stop_launch(session["session_id"])
```

**Session States**:
- `STARTING`: Launch is initializing
- `RUNNING`: Nodes are active
- `PAUSED`: Session suspended
- `STOPPING`: Shutting down
- `TERMINATED`: Fully stopped
- `ERROR`: Failed state

## Safety Concepts

### Multi-Layer Safety

The system implements multiple safety layers:

1. **MCP Validation**: Input validation at API level
2. **Autoware Safety**: Built-in Autoware safety checks
3. **Hardware Limits**: Physical constraints enforcement
4. **Emergency Stop**: Always available override

### Minimum Risk Maneuver (MRM)

Automatic safety behaviors when issues are detected:

```python
# Request emergency pullover
await client.request_mrm(behavior="pull_over")

# Available behaviors:
# - "comfortable_stop": Gradual stop
# - "emergency_stop": Immediate stop
# - "pull_over": Move to safe location
```

### Diagnostics Monitoring

Continuous health checking:

```python
diagnostics = await client.monitor_diagnostics()
if not diagnostics["healthy"]:
    print(f"Issues detected: {diagnostics['errors']}")
    # Trigger safety response
```

## Communication Patterns

### Synchronous Operations

Operations that complete immediately:

```python
# Get current state (synchronous)
state = await client.get_vehicle_state()
```

### Asynchronous Monitoring

Long-running operations with status updates:

```python
# Start driving (asynchronous)
await client.set_operation_mode("autonomous")

# Monitor progress
while True:
    route = await client.get_current_route()
    if route["state"] == "ARRIVED":
        break
    await asyncio.sleep(1)
```

### Event-Driven Responses

React to state changes:

```python
# Subscribe to diagnostics
async def on_diagnostic_change(data):
    if data["severity"] == "ERROR":
        await client.set_operation_mode("stop")
        
# Register callback
client.subscribe("diagnostics", on_diagnostic_change)
```

## Coordinate Systems

### Map Frame
- Global coordinate system
- Fixed to the map
- Used for localization and planning

### Base Link Frame
- Vehicle-centric coordinates
- Origin at vehicle center
- Used for sensor data

### Transform Example

```python
# Position in map frame
map_position = {"x": 3752.34, "y": 73736.09, "z": 0}

# Orientation as quaternion
orientation = {"x": 0, "y": 0, "z": -0.958, "w": 0.285}

# Combined pose
pose = {
    "position": map_position,
    "orientation": orientation
}
```

## Best Practices

### 1. Always Initialize First
```python
# Good ✅
await client.initialize_localization(initial_pose)
await asyncio.sleep(5)  # Wait for stability
await client.set_route(goal)

# Bad ❌
await client.set_route(goal)  # Will fail without localization
```

### 2. Check States Before Actions
```python
# Good ✅
mode = await client.monitor_operation_mode()
if mode["mode"] == "STOP":
    await client.set_operation_mode("autonomous")

# Bad ❌
await client.set_operation_mode("autonomous")  # May already be autonomous
```

### 3. Handle Errors Gracefully
```python
# Good ✅
try:
    result = await client.set_route(goal)
    if not result["success"]:
        logger.error(f"Route failed: {result['error']}")
        # Fallback behavior
except Exception as e:
    logger.error(f"Unexpected error: {e}")
    await client.set_operation_mode("stop")

# Bad ❌
result = await client.set_route(goal)  # Unchecked result
```

### 4. Monitor Continuously
```python
# Good ✅
async def monitor_health():
    while running:
        health = await client.health_check()
        if not health["healthy"]:
            await handle_unhealthy_state()
        await asyncio.sleep(1)

# Bad ❌
# Only checking once at startup
health = await client.health_check()
```

## Next Steps

Now that you understand the core concepts:

- Try the [Basic Usage](./basic-usage.md) examples
- Explore [Autonomous Driving](./autonomous-driving.md) scenarios
- Learn about [Launch Management](./launch-management.md)
- Dive into [Vehicle Control](./vehicle-control.md) details