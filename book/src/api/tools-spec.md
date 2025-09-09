# MCP Tools Specification

## Tool Categories

### System & Health
| Tool | Description | Returns |
|------|-------------|---------|
| `health_check` | Complete health status | System, ROS2, Autoware status |
| `get_system_status` | Resource usage | CPU, memory, disk, network |
| `verify_ros2_environment` | Environment check | ROS2 setup validation |
| `get_configuration` | MCP configuration | Server settings |

### ROS2 Operations
| Tool | Description | Returns |
|------|-------------|---------|
| `list_ros2_nodes` | List active nodes | Node names with namespaces |
| `list_ros2_topics` | List active topics | Topic names |
| `list_ros2_services` | List active services | Service names |
| `get_node_info` | Node details | Publishers, subscribers, services |
| `get_topic_info` | Topic details | Message type, pub/sub counts |
| `get_topic_frequency` | Topic Hz | Publishing frequency |
| `echo_topic_messages` | Capture messages | Topic message content |
| `call_ros2_service` | Call any service | Service response |
| `publish_to_topic` | Publish message | Success status |

### Vehicle Operations
| Tool | Description | Returns |
|------|-------------|---------|
| `set_operation_mode` | Change mode (stop/auto/manual) | Mode status |
| `monitor_operation_mode` | Current mode | Operation state |
| `get_vehicle_state` | Complete vehicle info | Dimensions, status, kinematics |

### Navigation
| Tool | Description | Returns |
|------|-------------|---------|
| `set_route` | Set goal with optional waypoints | Route status |
| `set_route_points` | Set waypoint route | Route status |
| `get_current_route` | Route progress | Route state |
| `initialize_localization` | Set initial pose | Localization status |
| `monitor_localization_state` | Localization quality | State metrics |

### Vehicle Control
| Tool | Description | Returns |
|------|-------------|---------|
| `send_velocity_command` | Set target velocity | Command ack |
| `send_acceleration_command` | Set acceleration | Command ack |
| `send_steering_command` | Set steering angle | Command ack |
| `send_pedals_command` | Set throttle/brake | Command ack |
| `monitor_motion_state` | Motion readiness | Motion state |

### Fail-Safe
| Tool | Description | Returns |
|------|-------------|---------|
| `request_mrm` | Request minimum risk maneuver | MRM status |
| `list_mrm_behaviors` | Available MRM types | Behavior list |
| `monitor_mrm_state` | Current MRM state | Active behaviors |

### Launch Management
| Tool | Description | Returns |
|------|-------------|---------|
| `start_launch` | Start launch file | Session ID, PID |
| `stop_launch` | Stop session | Success status |
| `pause_launch` | Pause session | Pause status |
| `resume_launch` | Resume session | Resume status |
| `restart_launch` | Restart session | New session ID |
| `list_launch_sessions` | All sessions | Session list |
| `get_session_status` | Session details | State, nodes, PIDs |
| `get_session_logs` | Session logs | stdout/stderr |
| `cleanup_orphans` | Clean orphaned processes | Cleanup info |

### Launch Generation
| Tool | Description | Returns |
|------|-------------|---------|
| `generate_launch_file` | Create launch file | File path |
| `validate_launch_file` | Validate syntax | Validation results |
| `generate_node_config` | Create node config | Config path |
| `generate_custom_node` | Create node template | Node file path |
| `test_launch_file` | Test launch file | Test results |
| `list_generated_files` | List AI-generated files | File list |

### Perception
| Tool | Description | Returns |
|------|-------------|---------|
| `capture_camera_view` | Capture camera image | Image path |
| `visualize_lidar_scene` | Create LiDAR visualization | Image path |
| `get_perception_snapshot` | Complete perception state | Multiple images |
| `analyze_driving_scene` | Scene analysis | Images, objects, recommendations |
| `get_detected_objects` | List detected objects | Object list with properties |

## Tool Usage Patterns

### Sequential Operations
```python
# Initialize -> Set Route -> Start Driving
await initialize_localization(pose)
await set_route(goal_pose)
await set_operation_mode("autonomous")
```

### Monitoring Loop
```python
# Monitor vehicle progress
while True:
    state = await get_vehicle_state()
    route = await get_current_route()
    if route["state"] == "ARRIVED":
        break
```

### Error Handling
```python
# All tools return success/error
result = await call_ros2_service(service, type, request)
if not result["success"]:
    print(f"Error: {result['error']}")
```

## Response Models

### Standard Response
```json
{
    "success": true,
    "data": {...},
    "error": null,
    "timestamp": "2024-01-01T00:00:00Z"
}
```

### Session Response
```json
{
    "session_id": "uuid",
    "main_pid": 12345,
    "pgid": 12345,
    "state": "running",
    "logs_path": "/path/to/logs"
}
```

### Vehicle State Response
```json
{
    "dimensions": {...},
    "status": {...},
    "kinematics": {
        "position": {"x": 0, "y": 0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        "velocity": {"linear": {...}, "angular": {...}}
    }
}
```