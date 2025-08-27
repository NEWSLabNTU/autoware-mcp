# Vehicle Control

Advanced vehicle control operations and techniques for managing Autoware-equipped vehicles.

## Control Modes

Autoware supports multiple operation modes for different driving scenarios:

### Stop Mode
The default safe mode where the vehicle is stationary and control commands are disabled.

```python
# Switch to stop mode
await client.set_operation_mode("stop")
```

### Autonomous Mode
Fully autonomous driving mode where Autoware controls the vehicle.

```python
# Prerequisites:
# 1. Localization must be initialized
# 2. Route must be set and valid
# 3. All safety checks must pass

await client.set_operation_mode("autonomous")

# Monitor autonomous driving
while True:
    mode = await client.monitor_operation_mode()
    if mode["mode"] != "autonomous":
        print("Mode changed, stopping monitoring")
        break
    
    state = await client.get_vehicle_state()
    print(f"Speed: {state['status']['velocity']:.2f} m/s")
    await asyncio.sleep(0.5)
```

### Manual Mode (Local)
Local manual control mode for debugging and testing.

```python
await client.set_operation_mode("local")
```

### Remote Mode
Remote control mode for teleoperation.

```python
await client.set_operation_mode("remote")
```

## Control Commands

### Velocity Control

Direct velocity commands for testing and manual control:

```python
# Send velocity command (m/s)
await client.send_velocity_command(velocity=2.0)  # 2 m/s forward
await client.send_velocity_command(velocity=-1.0)  # 1 m/s reverse
```

### Acceleration Control

Control vehicle acceleration:

```python
# Send acceleration command (m/s²)
await client.send_acceleration_command(acceleration=1.5)  # Accelerate
await client.send_acceleration_command(acceleration=-2.0)  # Decelerate
```

### Steering Control

Direct steering angle control:

```python
# Send steering command (radians)
await client.send_steering_command(steering_angle=0.2)  # Turn left
await client.send_steering_command(steering_angle=-0.2)  # Turn right
await client.send_steering_command(steering_angle=0.0)  # Straight
```

### Pedal Control

Low-level throttle and brake control:

```python
# Control throttle and brake (0-1 range)
await client.send_pedals_command(throttle=0.3, brake=0.0)  # 30% throttle
await client.send_pedals_command(throttle=0.0, brake=0.5)  # 50% brake
await client.send_pedals_command(throttle=0.0, brake=0.0)  # Coast
```

## Safety Features

### Minimum Risk Maneuver (MRM)

Request emergency maneuvers when safety is compromised:

```python
# List available MRM behaviors
behaviors = await client.list_mrm_behaviors()
print(f"Available behaviors: {behaviors}")

# Request emergency stop
await client.request_mrm(
    behavior="emergency_stop",
    reason="Obstacle detected"
)

# Monitor MRM state
mrm_state = await client.monitor_mrm_state()
if mrm_state["active"]:
    print(f"MRM active: {mrm_state['behavior']}")
```

### Motion State Monitoring

Continuously monitor vehicle motion readiness:

```python
async def monitor_motion():
    while True:
        motion = await client.monitor_motion_state()
        
        if not motion["ready"]:
            print("⚠️ Motion not ready:")
            for issue in motion["issues"]:
                print(f"  - {issue}")
        
        if motion["emergency_stopped"]:
            print("🛑 Emergency stop active!")
            break
        
        await asyncio.sleep(1)
```

## Control Patterns

### Safe Mode Transition

```python
async def safe_mode_change(target_mode, timeout=10):
    """Safely transition to a new operation mode."""
    try:
        # Check current mode
        current = await client.monitor_operation_mode()
        if current["mode"] == target_mode:
            return True
        
        # Verify prerequisites
        if target_mode == "autonomous":
            # Check localization
            loc_state = await client.monitor_localization_state()
            if not loc_state["initialized"]:
                print("Localization not initialized")
                return False
            
            # Check route
            route = await client.get_current_route()
            if route["state"] != 2:  # Not SET
                print("Route not set")
                return False
        
        # Request mode change
        result = await client.set_operation_mode(
            mode=target_mode,
            transition_time=timeout
        )
        
        if result["success"]:
            print(f"Transitioned to {target_mode} mode")
            return True
        else:
            print(f"Mode transition failed: {result.get('message', 'Unknown error')}")
            return False
            
    except Exception as e:
        print(f"Error during mode transition: {e}")
        return False
```

### Smooth Velocity Control

```python
async def smooth_velocity_change(target_velocity, duration=5.0):
    """Gradually change velocity over specified duration."""
    state = await client.get_vehicle_state()
    current_velocity = state["status"]["velocity"]
    
    steps = 20
    step_duration = duration / steps
    velocity_increment = (target_velocity - current_velocity) / steps
    
    for i in range(steps):
        intermediate_velocity = current_velocity + (velocity_increment * (i + 1))
        await client.send_velocity_command(velocity=intermediate_velocity)
        await asyncio.sleep(step_duration)
    
    print(f"Reached target velocity: {target_velocity} m/s")
```

### Emergency Stop Pattern

```python
async def emergency_stop():
    """Perform immediate emergency stop."""
    try:
        # Multiple redundant stop commands
        await asyncio.gather(
            client.set_operation_mode("stop"),
            client.send_velocity_command(velocity=0),
            client.send_pedals_command(throttle=0, brake=1),
            client.request_mrm(behavior="emergency_stop", reason="Manual trigger")
        )
        print("Emergency stop executed")
    except Exception as e:
        print(f"Emergency stop error: {e}")
        # Last resort: direct service call
        await client.call_ros2_service(
            "/api/motion/emergency_stop",
            "std_srvs/srv/Trigger",
            {}
        )
```

## Advanced Control

### Cooperation Commands

Manage vehicle cooperation with infrastructure and other vehicles:

```python
# Get current cooperation policies
policies = await client.get_cooperation_policies()
print(f"Active policies: {policies}")

# Set cooperation policies
await client.set_cooperation_policies({
    "enable_v2x": True,
    "follow_traffic_lights": True,
    "yield_to_pedestrians": True
})

# Send cooperation commands
await client.send_cooperation_commands({
    "request_lane_change": "left",
    "signal_intention": "turning_right"
})
```

### Direct Service Control

For advanced users, direct ROS2 service calls provide fine-grained control:

```python
# Enable Autoware control
await client.call_ros2_service(
    "/api/operation_mode/enable_autoware_control",
    "std_srvs/srv/Trigger",
    {}
)

# Accept motion start
await client.call_ros2_service(
    "/api/motion/accept_start",
    "std_srvs/srv/Trigger",
    {}
)

# Custom control command
await client.call_ros2_service(
    "/control/command_gate/engage",
    "std_srvs/srv/Trigger",
    {}
)
```

## Best Practices

1. **Always verify mode transitions** before sending control commands
2. **Monitor vehicle state continuously** during manual control
3. **Implement timeout mechanisms** for all control operations
4. **Use emergency stop patterns** for safety-critical applications
5. **Log all control commands** for debugging and analysis
6. **Test in simulation first** before real vehicle control
7. **Implement redundant safety checks** at multiple levels

## Common Issues

### Vehicle Not Responding to Commands

1. Check operation mode - must not be in STOP mode
2. Verify control engagement status
3. Check for active MRM or emergency stop
4. Ensure localization is stable

### Erratic Vehicle Behavior

1. Reduce control command frequency
2. Check for conflicting control sources
3. Verify sensor data quality
4. Review control parameter tuning

## Next Steps

- Learn about [Autonomous Driving](./autonomous-driving.md) workflows
- Explore [Launch Management](./launch-management.md) for complex scenarios
- Study [MRM and Safety](../advanced/mrm-safety.md) systems