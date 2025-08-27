# Autoware MCP Quick Reference

## Common Operations

### 🚀 Start Autonomous Driving

```python
# 1. Initialize localization
await initialize_localization({
    "pose": {
        "position": {"x": 0, "y": 0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
    }
})

# 2. Set route to destination
await set_route({
    "goal_pose": {
        "position": {"x": 100, "y": 50, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
    }
})

# 3. Change to autonomous mode
await set_operation_mode({"mode": "autonomous"})

# 4. Enable control
await call_ros2_service(
    "/api/operation_mode/enable_autoware_control",
    "std_srvs/srv/Trigger",
    {}
)

# 5. Accept start
await call_ros2_service(
    "/api/motion/accept_start",
    "std_srvs/srv/Trigger",
    {}
)
```

### 🛑 Stop Operations

```python
# Emergency stop
await request_mrm({
    "behavior": "emergency_stop",
    "reason": "User requested stop"
})

# OR Normal stop
await set_operation_mode({"mode": "stop"})
```

### 📍 Route Management

| Action | Method |
|--------|--------|
| Set route | `set_route({"goal_pose": {...}})` |
| Set route with waypoints | `set_route_points([waypoint1, waypoint2, ...])` |
| Clear route | `call_ros2_service("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {})` |
| Get route state | `echo_topic_messages("/api/routing/state", 1)` |

### 🚗 Vehicle Control

| Action | Method |
|--------|--------|
| Set velocity | `send_velocity_command(velocity=5.0)` |
| Set acceleration | `send_acceleration_command(acceleration=1.0)` |
| Set steering | `send_steering_command(steering_angle=0.1)` |
| Apply brakes | `send_pedals_command(throttle=0, brake=0.5)` |

### 📊 Status Monitoring

| Information | Method |
|------------|--------|
| Vehicle state | `get_vehicle_state()` |
| Operation mode | `monitor_operation_mode()` |
| Localization | `monitor_localization_state()` |
| Motion state | `monitor_motion_state()` |
| System health | `health_check()` |
| Autoware status | `check_autoware_status()` |

### 🔍 Discovery

```python
# List all available services
services = await list_ros2_services()

# List all topics
topics = await list_ros2_topics()

# Get topic info
info = await get_topic_info("/api/operation_mode/state")

# Measure topic frequency
freq = await get_topic_frequency("/tf", duration=5.0)
```

## ROS2 Service/Topic Reference

### Operation Mode Services

| Service | Type | Purpose |
|---------|------|---------|
| `/api/operation_mode/change_to_autonomous` | `autoware_adapi_v1_msgs/srv/ChangeOperationMode` | Switch to autonomous |
| `/api/operation_mode/change_to_stop` | `autoware_adapi_v1_msgs/srv/ChangeOperationMode` | Switch to stop |
| `/api/operation_mode/enable_autoware_control` | `std_srvs/srv/Trigger` | Enable control |
| `/api/operation_mode/disable_autoware_control` | `std_srvs/srv/Trigger` | Disable control |

### Route Services

| Service | Type | Purpose |
|---------|------|---------|
| `/api/routing/set_route` | `autoware_adapi_v1_msgs/srv/SetRoute` | Set route to goal |
| `/api/routing/set_route_points` | `autoware_adapi_v1_msgs/srv/SetRoutePoints` | Set route with waypoints |
| `/api/routing/clear_route` | `autoware_adapi_v1_msgs/srv/ClearRoute` | Clear current route |

### Motion Services

| Service | Type | Purpose |
|---------|------|---------|
| `/api/motion/accept_start` | `std_srvs/srv/Trigger` | Accept motion start |

### Localization Services

| Service | Type | Purpose |
|---------|------|---------|
| `/api/localization/initialize` | `autoware_adapi_v1_msgs/srv/InitializeLocalization` | Set initial pose |

### Important Topics

| Topic | Purpose | Key Fields |
|-------|---------|------------|
| `/api/operation_mode/state` | Current operation mode | `mode`, `is_autoware_control_enabled` |
| `/api/routing/state` | Route status | `state`, `route_segments` |
| `/api/motion/state` | Motion readiness | `state`, `motion_ready` |
| `/api/localization/initialization_state` | Localization status | `state` (0=uninit, 1=initializing, 2=initialized) |
| `/api/fail_safe/mrm_state` | MRM status | `state`, `behavior` |
| `/vehicle/status/velocity_status` | Vehicle velocity | `longitudinal_velocity`, `lateral_velocity` |
| `/localization/kinematic_state` | Vehicle pose | `pose`, `twist` |

## State Enumerations

### Operation Modes
```python
OPERATION_MODES = {
    0: "UNKNOWN",
    1: "STOP",
    2: "AUTONOMOUS",
    3: "LOCAL",
    4: "REMOTE"
}
```

### Localization States
```python
LOCALIZATION_STATES = {
    0: "UNINITIALIZED",
    1: "INITIALIZING",
    2: "INITIALIZED"
}
```

### Motion States
```python
MOTION_STATES = {
    0: "STOPPED",
    1: "STARTING",
    2: "MOVING"
}
```

### MRM Behaviors
```python
MRM_BEHAVIORS = [
    "comfortable_stop",
    "emergency_stop",
    "pull_over"
]
```

## Generic Tool Examples

### Call Any Service
```python
result = await call_ros2_service(
    service_name="/your/service",
    service_type="your_msgs/srv/YourService",
    request={"field1": "value1", "field2": 123}
)
```

### Publish to Any Topic
```python
await publish_to_topic(
    topic_name="/your/topic",
    message_type="your_msgs/msg/YourMessage",
    message={"field1": "value1", "field2": 123}
)
```

### Read from Any Topic
```python
messages = await echo_topic_messages(
    topic_name="/your/topic",
    count=5  # Get 5 messages
)
```

## Typical Workflow

1. **Check system health**
   ```python
   health = await health_check()
   ```

2. **Initialize localization**
   ```python
   await initialize_localization({"pose": initial_pose})
   ```

3. **Set destination**
   ```python
   await set_route({"goal_pose": destination})
   ```

4. **Start autonomous driving**
   ```python
   await set_operation_mode({"mode": "autonomous"})
   await call_ros2_service("/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {})
   await call_ros2_service("/api/motion/accept_start", "std_srvs/srv/Trigger", {})
   ```

5. **Monitor progress**
   ```python
   while driving:
       state = await get_vehicle_state()
       route = await echo_topic_messages("/api/routing/state", 1)
       # Check if arrived, handle events, etc.
   ```

6. **Stop when done**
   ```python
   await set_operation_mode({"mode": "stop"})
   ```

## Tips

- Use `list_ros2_services()` to discover available services
- Use `get_topic_info()` to find message types
- Use wrapper tools for safety-critical operations
- Use generic tools for simple service calls
- Monitor state changes after commands with topic echo
- Check `health_check()` before starting operations