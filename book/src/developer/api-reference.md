# Autoware MCP API Documentation

## Overview

The Autoware MCP Server provides two types of tools:

1. **Generic ROS2 Tools** - Direct access to any ROS2 topic or service
2. **High-Value Wrapper Tools** - Complex operations with state management, validation, and safety checks

## Architecture

```
┌─────────────────────────────────────────────┐
│          Application Layer (Thin)           │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │   Safety-Critical Operations        │    │
│  │   - initialize_localization         │    │
│  │   - set_operation_mode              │    │
│  │   - request_mrm                     │    │
│  └─────────────────────────────────────┘    │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │   Complex State Management          │    │
│  │   - set_route / set_route_points    │    │
│  │   - health_check                    │    │
│  │   - check_autoware_status           │    │
│  └─────────────────────────────────────┘    │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│         Foundation Layer (Generic)          │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │   Direct ROS2 Access                │    │
│  │   - call_ros2_service               │    │
│  │   - publish_to_topic                │    │
│  │   - echo_topic_messages             │    │
│  └─────────────────────────────────────┘    │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │   ROS2 Discovery                    │    │
│  │   - list_ros2_nodes                 │    │
│  │   - list_ros2_topics                │    │
│  │   - list_ros2_services              │    │
│  └─────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
```

## Generic ROS2 Tools

### call_ros2_service

Call any ROS2 service directly.

**Parameters:**
- `service_name` (str): Full service name (e.g., "/api/routing/clear_route")
- `service_type` (str): Service type (e.g., "std_srvs/srv/Trigger")
- `request` (dict): Service request data as JSON

**Examples:**
```python
# Clear route
await call_ros2_service(
    "/api/routing/clear_route",
    "autoware_adapi_v1_msgs/srv/ClearRoute",
    {}
)

# Enable Autoware control
await call_ros2_service(
    "/api/operation_mode/enable_autoware_control",
    "std_srvs/srv/Trigger",
    {}
)

# Accept motion start
await call_ros2_service(
    "/api/motion/accept_start",
    "std_srvs/srv/Trigger",
    {}
)
```

### publish_to_topic

Publish a message to any ROS2 topic.

**Parameters:**
- `topic_name` (str): Topic name to publish to
- `message_type` (str): Message type (e.g., "geometry_msgs/msg/Twist")
- `message` (dict): Message data to publish as JSON

**Examples:**
```python
# Publish velocity command
await publish_to_topic(
    "/cmd_vel",
    "geometry_msgs/msg/Twist",
    {
        "linear": {"x": 1.0, "y": 0, "z": 0},
        "angular": {"x": 0, "y": 0, "z": 0.5}
    }
)

# Publish initial pose
await publish_to_topic(
    "/initialpose",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
    {
        "pose": {
            "pose": {
                "position": {"x": 0, "y": 0, "z": 0},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
            },
            "covariance": [0.25] * 36
        }
    }
)
```

### echo_topic_messages

Capture and return messages from a ROS2 topic.

**Parameters:**
- `topic_name` (str): Name of the ROS2 topic
- `count` (int): Number of messages to capture (default: 1)

**Example:**
```python
# Get current operation mode
state = await echo_topic_messages("/api/operation_mode/state", 1)

# Get route state
route = await echo_topic_messages("/api/routing/state", 1)
```

### ROS2 Discovery Tools

- `list_ros2_nodes()` - List all active ROS2 nodes
- `list_ros2_topics()` - List all active ROS2 topics
- `list_ros2_services()` - List all active ROS2 services
- `get_topic_info(topic_name)` - Get topic details (message type, publishers, subscribers)
- `get_topic_frequency(topic_name, duration)` - Measure topic publishing rate
- `get_node_info(node_name)` - Get detailed information about a node

## High-Value Wrapper Tools

### Safety-Critical Operations

#### set_operation_mode

Change vehicle operation mode with state validation and transitions.

**Parameters:**
- `request`: OperationModeRequest with:
  - `mode`: One of "stop", "autonomous", "local", "remote"
  - `transition_time`: Maximum time to wait for transition (default: 10.0 seconds)

**Returns:** OperationModeResponse with success status and current mode

#### initialize_localization

Initialize localization with a pose - critical for safe autonomous operation.

**Parameters:**
- `request`: LocalizationRequest with:
  - `pose`: Initial pose with position and orientation
  - `pose_with_covariance`: Optional covariance matrix

**Returns:** LocalizationResponse with success status and initialization state

#### request_mrm

Request a Minimum Risk Maneuver for safety-critical situations.

**Parameters:**
- `request`: MRMRequest with:
  - `behavior`: MRM behavior type ("comfortable_stop", "emergency_stop", "pull_over")
  - `reason`: Optional reason for MRM request

**Returns:** MRMResponse with success status and current MRM state

### Complex State Management

#### health_check

Get comprehensive health status aggregating multiple data sources.

**Returns:** HealthResponse with:
- System resources (CPU, memory, disk)
- ROS2 status and active nodes/topics
- Autoware workspace configuration
- Overall health status

#### check_autoware_status

Analyze ROS2 graph to identify and categorize Autoware components.

**Returns:** Dict with:
- Active Autoware nodes by category (perception, planning, control, etc.)
- Autoware-specific topics
- Component health status

#### set_route / set_route_points

Set navigation route with complex message construction and validation.

**Parameters:**
- `request`: RouteRequest with:
  - `goal_pose`: Goal pose with position and orientation
  - `waypoints`: Optional list of waypoints
  - `option`: Optional route options

**Returns:** RouteResponse with success status and route information

### Vehicle State

#### get_vehicle_state

Get comprehensive vehicle state in a single call.

**Returns:** Dict with:
- `dimensions`: Vehicle physical dimensions (length, width, height, wheelbase)
- `status`: Current gear, speed, steering angle
- `kinematics`: Position, orientation, velocities
- `timestamp`: When the data was collected

### Vehicle Control

#### send_pedals_command

Send pedals control command with throttle/brake abstraction.

**Parameters:**
- `throttle`: Throttle position (0-1)
- `brake`: Brake position (0-1)

**Returns:** Command acknowledgment

#### send_velocity_command

Send velocity control command.

**Parameters:**
- `velocity`: Target velocity in m/s (-10 to 50)

**Returns:** Command acknowledgment

#### send_acceleration_command

Send acceleration control command.

**Parameters:**
- `acceleration`: Target acceleration in m/s² (-5 to 3)

**Returns:** Command acknowledgment

#### send_steering_command

Send steering control command.

**Parameters:**
- `steering_angle`: Steering angle in radians (-0.7 to 0.7)

**Returns:** Command acknowledgment

### Monitoring Tools

#### monitor_operation_mode

Monitor current operation mode state.

**Returns:** Current operation mode and control state

#### monitor_localization_state

Monitor localization initialization state.

**Returns:** Current localization state and quality metrics

#### monitor_motion_state

Monitor current motion state.

**Returns:** Current motion state and readiness

#### monitor_mrm_state

Monitor current MRM state.

**Returns:** Current MRM state and active behaviors

#### monitor_diagnostics

Get real-time diagnostics status.

**Returns:** Comprehensive diagnostics information

#### monitor_system_heartbeat

Monitor system heartbeat.

**Returns:** System heartbeat status

## Common ROS2 Interface Patterns

### Operation Mode Management

```python
# Check current mode
state = await echo_topic_messages("/api/operation_mode/state", 1)

# Change to autonomous (using wrapper for safety)
await set_operation_mode({"mode": "autonomous"})

# Enable control (using generic tool)
await call_ros2_service(
    "/api/operation_mode/enable_autoware_control",
    "std_srvs/srv/Trigger",
    {}
)
```

### Route Management

```python
# Set route (using wrapper for complex logic)
await set_route({
    "goal_pose": {
        "position": {"x": 100, "y": 50, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
    }
})

# Clear route (using generic tool)
await call_ros2_service(
    "/api/routing/clear_route",
    "autoware_adapi_v1_msgs/srv/ClearRoute",
    {}
)

# Check route state
route_state = await echo_topic_messages("/api/routing/state", 1)
```

### Motion Control

```python
# Accept start request (using generic tool)
await call_ros2_service(
    "/api/motion/accept_start",
    "std_srvs/srv/Trigger",
    {}
)

# Check motion state
motion_state = await echo_topic_messages("/api/motion/state", 1)
```

## State Enumerations

### Operation Modes
- 0: UNKNOWN
- 1: STOP
- 2: AUTONOMOUS
- 3: LOCAL
- 4: REMOTE

### Localization States
- 0: UNINITIALIZED
- 1: INITIALIZING
- 2: INITIALIZED

### Motion States
- 0: STOPPED
- 1: STARTING
- 2: MOVING

## Migration Guide

### From Old Wrapper Tools to Generic Tools

| Old Tool | New Method |
|----------|------------|
| `clear_route()` | `call_ros2_service("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {})` |
| `accept_start_request()` | `call_ros2_service("/api/motion/accept_start", "std_srvs/srv/Trigger", {})` |
| `enable_autoware_control()` | `call_ros2_service("/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {})` |
| `disable_autoware_control()` | `call_ros2_service("/api/operation_mode/disable_autoware_control", "std_srvs/srv/Trigger", {})` |
| `get_current_route()` | `echo_topic_messages("/api/routing/state", 1)` |
| `monitor_vehicle_status()` | `get_vehicle_state()` (returns all vehicle data) |
| `monitor_vehicle_kinematics()` | `get_vehicle_state()` (returns all vehicle data) |
| `get_vehicle_dimensions()` | `get_vehicle_state()` (returns all vehicle data) |

## Best Practices

1. **Use generic tools for simple operations** - Direct service calls and topic echoes
2. **Use wrapper tools for complex operations** - Safety-critical operations, state management
3. **Discover available interfaces** - Use `list_ros2_services()` and `list_ros2_topics()`
4. **Check service types** - Use `ros2 service type <service_name>` command externally if needed
5. **Monitor state changes** - Use topic echo after service calls to verify state changes

## Error Handling

All tools return structured responses with:
- `success`: Boolean indicating if the operation succeeded
- `error`: Error message if operation failed (optional)
- `data`/`response`: Operation-specific data

Example error handling:
```python
result = await call_ros2_service(
    "/api/some/service",
    "some_msgs/srv/SomeService",
    {"param": "value"}
)

if result["success"]:
    print(f"Success: {result['response']}")
else:
    print(f"Failed: {result['error']}")
```