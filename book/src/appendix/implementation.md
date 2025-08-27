# MCP Tools Refactoring Implementation Plan

## Quick Summary

Based on the architecture analysis, here's the implementation plan for refactoring the MCP tools to eliminate redundancy and focus on high-value abstractions.

## Phase 1: Add Generic ROS2 Tools (Priority: HIGH)

### New Tools to Implement

```python
# 1. Generic Service Caller
@mcp.tool()
async def call_ros2_service(
    service_name: str = Field(..., description="Full service name (e.g., /api/routing/clear_route)"),
    service_type: str = Field(..., description="Service type (e.g., std_srvs/srv/Trigger)"),
    request: Dict[str, Any] = Field(default={}, description="Service request data")
) -> Dict[str, Any]:
    """
    Call any ROS2 service directly.
    
    Examples:
    - Clear route: call_ros2_service("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {})
    - Enable control: call_ros2_service("/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {})
    """

# 2. Generic Topic Publisher
@mcp.tool()
async def publish_to_topic(
    topic_name: str = Field(..., description="Topic name to publish to"),
    message_type: str = Field(..., description="Message type (e.g., geometry_msgs/msg/Twist)"),
    message: Dict[str, Any] = Field(..., description="Message data to publish")
) -> bool:
    """Publish a message to any ROS2 topic."""

# 3. Enhanced Topic Subscriber
@mcp.tool()
async def subscribe_to_topic(
    topic_name: str = Field(..., description="Topic to subscribe to"),
    duration: float = Field(default=5.0, description="How long to collect messages"),
    max_messages: int = Field(default=10, description="Maximum messages to collect")
) -> List[Dict[str, Any]]:
    """Subscribe to a topic and collect messages over time."""
```

## Phase 2: Categorize Existing Tools

### Keep These High-Value Tools ✅

| Tool | Reason | Enhancement Needed |
|------|--------|-------------------|
| `health_check` | Complex aggregation | None |
| `check_autoware_status` | Domain-specific analysis | None |
| `set_operation_mode` | State management + validation | Add state transition validation |
| `set_route` / `set_route_points` | Complex message construction | Add coordinate validation |
| `initialize_localization` | Critical safety operation | Add retry logic |
| `request_mrm` | Safety-critical | Add behavior validation |
| `send_pedals_command` | Useful abstraction | None |

### Deprecate These Low-Value Tools ⚠️

Mark as deprecated but keep for backward compatibility:

| Tool | Replace With | Migration Guide |
|------|-------------|----------------|
| `clear_route` | `call_ros2_service("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {})` | Direct service call |
| `accept_start_request` | `call_ros2_service("/api/motion/accept_start", "std_srvs/srv/Trigger", {})` | Direct service call |
| `get_current_route` | `echo_topic_messages("/api/routing/state", 1)` | Direct topic read |
| `monitor_motion_state` | `echo_topic_messages("/api/motion/state", 1)` | Direct topic read |
| `enable_autoware_control` | `call_ros2_service("/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {})` | Direct service call |
| `disable_autoware_control` | `call_ros2_service("/api/operation_mode/disable_autoware_control", "std_srvs/srv/Trigger", {})` | Direct service call |

### Refactor These Tools 🔧

Combine or enhance for better value:

| Current Tools | New Tool | Description |
|--------------|----------|-------------|
| `monitor_vehicle_status`, `monitor_vehicle_kinematics`, `get_vehicle_dimensions` | `get_vehicle_state` | Unified vehicle state with all data |
| `send_velocity_command`, `send_acceleration_command`, `send_steering_command` | `send_control_command` | Unified control interface |
| `monitor_operation_mode` | Include in `get_system_state` | Part of overall state |

## Phase 3: Documentation Updates

### Create Quick Reference Guide

```markdown
# Autoware ROS2 Interface Quick Reference

## Common Operations

### Operation Mode Management
| Action | Method |
|--------|--------|
| Check mode | `echo_topic_messages("/api/operation_mode/state")` |
| Change to autonomous | `call_ros2_service("/api/operation_mode/change_to_autonomous", ...)` |
| Enable control | `call_ros2_service("/api/operation_mode/enable_autoware_control", ...)` |

### Route Management  
| Action | Method |
|--------|--------|
| Set route | Use `set_route()` tool (complex logic) |
| Clear route | `call_ros2_service("/api/routing/clear_route", ...)` |
| Check route | `echo_topic_messages("/api/routing/state")` |

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
```

## Phase 4: Testing Strategy

### Test Generic Tools
1. Test `call_ros2_service` with various service types
2. Test `publish_to_topic` with different message types
3. Verify error handling for invalid services/topics

### Regression Testing
1. Ensure deprecated tools still work
2. Verify high-value tools maintain functionality
3. Test new unified tools

## Migration Timeline

```
Week 1-2: Implement generic ROS2 tools
Week 3: Test and document generic tools
Week 4: Mark tools as deprecated with warnings
Week 5-6: Refactor moderate-value tools
Week 7: Update documentation and examples
Week 8: Release v2.0 with deprecation notices
Week 12: Remove deprecated tools in v3.0
```

## Code Example: Using New Architecture

### Before (Many Specific Tools)
```python
# Old way - specific tool for each operation
await mcp.enable_autoware_control()
await mcp.clear_route()
await mcp.accept_start_request()
state = await mcp.monitor_operation_mode()
```

### After (Generic Tools + High-Value Wrappers)
```python
# New way - generic tools for simple operations
await mcp.call_ros2_service("/api/operation_mode/enable_autoware_control", "std_srvs/srv/Trigger", {})
await mcp.call_ros2_service("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", {})
await mcp.call_ros2_service("/api/motion/accept_start", "std_srvs/srv/Trigger", {})
state = await mcp.echo_topic_messages("/api/operation_mode/state", 1)

# High-value wrappers for complex operations
await mcp.set_route(goal_pose)  # Still wrapped - complex logic
await mcp.initialize_localization(initial_pose)  # Still wrapped - safety critical
```

## Benefits Summary

1. **70% reduction** in tool count (from ~40 to ~12)
2. **100% coverage** - any ROS2 interface accessible
3. **Clearer architecture** - obvious value in each tool
4. **Easier maintenance** - fewer tools to update
5. **Better flexibility** - users can access new Autoware features immediately

## Next Steps

1. Review and approve this plan
2. Implement generic ROS2 tools
3. Add deprecation warnings to low-value tools
4. Update documentation
5. Release with migration guide