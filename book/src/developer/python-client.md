# Python Client Library

Guide for using the Autoware MCP Python client library.

## Installation

```bash
pip install autoware-mcp
```

Or in your project:
```bash
uv add autoware-mcp
```

## Quick Start

```python
import asyncio
from autoware_mcp.client import AutowareMCPClient

async def main():
    # Initialize client
    client = AutowareMCPClient()
    
    # Connect to server
    await client.connect()
    
    # Check health
    health = await client.health_check()
    print(f"System healthy: {health['healthy']}")
    
    # Disconnect
    await client.disconnect()

asyncio.run(main())
```

## Client Configuration

### Connection Options

```python
# Default connection (stdio)
client = AutowareMCPClient()

# Custom server command
client = AutowareMCPClient(
    server_command=["uv", "run", "autoware-mcp"],
    server_cwd="/path/to/autoware-mcp"
)

# With environment variables
client = AutowareMCPClient(
    env={
        "ROS_DOMAIN_ID": "42",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"
    }
)

# With timeout
client = AutowareMCPClient(
    connection_timeout=30,  # seconds
    request_timeout=10      # seconds per request
)
```

### Async Context Manager

```python
async def main():
    async with AutowareMCPClient() as client:
        # Client is automatically connected and disconnected
        health = await client.health_check()
        print(health)
```

## Core Operations

### System Health

```python
# Check overall health
health = await client.health_check()
if not health['healthy']:
    print("System issues detected")
    
# Get detailed system status
status = await client.get_system_status()
print(f"CPU: {status['cpu']['percent']}%")
print(f"Memory: {status['memory']['percent']}%")

# Verify ROS2 environment
env = await client.verify_ros2_environment()
print(f"ROS2 Distro: {env['ros_distro']}")
```

### ROS2 Inspection

```python
# List nodes
nodes = await client.list_ros2_nodes()
print(f"Active nodes: {nodes['count']}")

# List topics
topics = await client.list_ros2_topics()
for topic in topics['topics']:
    info = await client.get_topic_info(topic)
    print(f"{topic}: {info['type']}")

# Monitor topic frequency
freq = await client.get_topic_frequency("/tf", duration=5)
print(f"TF rate: {freq['average_rate']} Hz")

# Echo topic messages
msgs = await client.echo_topic_messages("/localization/kinematic_state", count=3)
for msg in msgs['messages']:
    print(f"Position: {msg['pose']['pose']['position']}")
```

## Autonomous Driving

### Complete Workflow

```python
async def autonomous_drive_example():
    async with AutowareMCPClient() as client:
        # 1. Initialize localization
        initial_pose = {
            "position": {"x": 3752.34, "y": 73736.09, "z": 19.34},
            "orientation": {"x": 0, "y": 0, "z": -0.958, "w": 0.285}
        }
        
        result = await client.initialize_localization(initial_pose)
        if not result['success']:
            raise Exception("Localization failed")
        
        # Wait for convergence
        await asyncio.sleep(5)
        
        # 2. Clear any existing route
        await client.call_ros2_service(
            "/api/routing/clear_route",
            "std_srvs/srv/Trigger",
            {}
        )
        
        # 3. Set route to goal
        goal_pose = {
            "position": {"x": 3798.71, "y": 73775.45, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0.862, "w": 0.507}
        }
        
        route_result = await client.set_route({"goal_pose": goal_pose})
        if not route_result['success']:
            raise Exception("Route planning failed")
        
        # 4. Verify route
        route = await client.get_current_route()
        if route['state'] != 2:  # Not SET
            raise Exception("Route not ready")
        
        # 5. Engage autonomous mode
        mode_result = await client.set_operation_mode("autonomous")
        if not mode_result['success']:
            raise Exception("Failed to engage autonomous mode")
        
        print("Vehicle is driving autonomously!")
        
        # 6. Monitor progress
        while True:
            state = await client.get_vehicle_state()
            speed = state['status']['velocity']
            position = state['kinematics']['pose']['position']
            
            print(f"Speed: {speed:.1f} m/s, Position: ({position['x']:.1f}, {position['y']:.1f})")
            
            route = await client.get_current_route()
            if route['state'] == 3:  # ARRIVED
                print("Destination reached!")
                break
            
            await asyncio.sleep(1)
        
        # 7. Stop vehicle
        await client.set_operation_mode("stop")
```

### Multi-Waypoint Navigation

```python
async def navigate_waypoints(client, waypoints):
    """Navigate through multiple waypoints."""
    
    # Set route with waypoints
    result = await client.set_route_points(waypoints)
    if not result['success']:
        raise Exception("Failed to set waypoint route")
    
    # Engage autonomous mode
    await client.set_operation_mode("autonomous")
    
    # Monitor progress
    for i, waypoint in enumerate(waypoints):
        print(f"Heading to waypoint {i+1}/{len(waypoints)}")
        
        while True:
            state = await client.get_vehicle_state()
            pos = state['kinematics']['pose']['position']
            
            # Calculate distance to waypoint
            dx = waypoint['position']['x'] - pos['x']
            dy = waypoint['position']['y'] - pos['y']
            distance = (dx**2 + dy**2) ** 0.5
            
            if distance < 5.0:  # Within 5 meters
                print(f"Reached waypoint {i+1}")
                break
            
            await asyncio.sleep(1)
    
    print("All waypoints visited!")
```

## Vehicle Control

### Operation Modes

```python
# Change modes with verification
async def change_mode_safely(client, target_mode):
    # Check current mode
    current = await client.monitor_operation_mode()
    if current['mode'] == target_mode:
        return True
    
    # Verify prerequisites for autonomous
    if target_mode == "autonomous":
        loc = await client.monitor_localization_state()
        if not loc['initialized']:
            print("Localization not ready")
            return False
        
        route = await client.get_current_route()
        if route['state'] != 2:
            print("Route not set")
            return False
    
    # Change mode
    result = await client.set_operation_mode(target_mode)
    return result['success']
```

### Direct Control Commands

```python
# Velocity control
await client.send_velocity_command(velocity=2.0)  # 2 m/s

# Acceleration control
await client.send_acceleration_command(acceleration=1.5)  # 1.5 m/s²

# Steering control
await client.send_steering_command(steering_angle=0.2)  # radians

# Pedal control
await client.send_pedals_command(throttle=0.3, brake=0.0)

# Combined control
async def smooth_stop(client, decel_time=5.0):
    """Smoothly stop the vehicle."""
    for i in range(50):
        brake_force = i / 50.0
        await client.send_pedals_command(throttle=0, brake=brake_force)
        await asyncio.sleep(decel_time / 50)
```

### Emergency Procedures

```python
async def emergency_stop(client):
    """Execute emergency stop with redundancy."""
    
    # Multiple stop commands in parallel
    await asyncio.gather(
        client.set_operation_mode("stop"),
        client.send_velocity_command(velocity=0),
        client.send_pedals_command(throttle=0, brake=1),
        client.request_mrm(
            behavior="emergency_stop",
            reason="Manual emergency trigger"
        ),
        return_exceptions=True  # Continue even if some fail
    )
    
    print("Emergency stop executed!")

async def recover_from_emergency(client):
    """Recover from emergency stop."""
    
    # Clear emergency state
    await client.call_ros2_service(
        "/api/motion/clear_emergency",
        "std_srvs/srv/Trigger",
        {}
    )
    
    # Reset controls
    await client.send_pedals_command(throttle=0, brake=0)
    
    # Re-verify route
    route = await client.get_current_route()
    if route['state'] == 1:  # UNSET
        # Re-plan route
        pass
    
    print("Ready to resume")
```

## Launch Management

### Managing Launch Sessions

```python
# Start a launch file
result = await client.start_launch(
    launch_file="perception.launch.py",
    parameters={"use_sim_time": "true"},
    launch_args=["--debug"]
)
session_id = result['session_id']
print(f"Started session: {session_id}")

# Monitor session
status = await client.get_session_status(session_id)
print(f"State: {status['state']}, Nodes: {status['node_count']}")

# Get logs
logs = await client.get_session_logs(session_id, lines=50)
for line in logs['lines']:
    print(line)

# Pause/Resume
await client.pause_launch(session_id)
await asyncio.sleep(5)
await client.resume_launch(session_id)

# Stop session
await client.stop_launch(session_id)
```

### Generating Launch Files

```python
# Generate perception pipeline
result = await client.generate_launch_file(
    name="custom_perception",
    components=["lidar_processor", "camera_detector", "fusion"],
    template="perception_pipeline",
    parameters={
        "lidar_range": 100.0,
        "camera_fps": 30
    }
)

launch_file = result['file_path']
print(f"Generated: {launch_file}")

# Validate before running
validation = await client.validate_launch_file(launch_file)
if validation['valid']:
    # Start the generated launch file
    session = await client.start_launch(launch_file)
```

## Monitoring and Diagnostics

### Continuous Monitoring

```python
async def monitor_system(client):
    """Comprehensive system monitoring."""
    
    async def monitor_health():
        while True:
            health = await client.health_check()
            if not health['healthy']:
                print(f"Health issue detected: {health}")
            await asyncio.sleep(10)
    
    async def monitor_diagnostics():
        while True:
            diag = await client.monitor_diagnostics()
            for warning in diag['warnings']:
                print(f"Warning: {warning['message']}")
            for error in diag['errors']:
                print(f"Error: {error['message']}")
            await asyncio.sleep(5)
    
    async def monitor_vehicle():
        while True:
            state = await client.get_vehicle_state()
            speed = state['status']['velocity']
            
            if speed > 10.0:  # Speed limit
                print(f"Speed limit exceeded: {speed:.1f} m/s")
            
            await asyncio.sleep(1)
    
    # Run all monitors concurrently
    await asyncio.gather(
        monitor_health(),
        monitor_diagnostics(),
        monitor_vehicle()
    )
```

### Performance Monitoring

```python
async def performance_monitor(client, duration=60):
    """Monitor performance metrics."""
    
    start_time = asyncio.get_event_loop().time()
    samples = []
    
    while asyncio.get_event_loop().time() - start_time < duration:
        # Collect metrics
        status = await client.get_system_status()
        
        sample = {
            "timestamp": asyncio.get_event_loop().time(),
            "cpu": status['cpu']['percent'],
            "memory": status['memory']['percent'],
            "nodes": len((await client.list_ros2_nodes())['nodes']),
            "topics": len((await client.list_ros2_topics())['topics'])
        }
        samples.append(sample)
        
        await asyncio.sleep(1)
    
    # Analyze
    avg_cpu = sum(s['cpu'] for s in samples) / len(samples)
    avg_memory = sum(s['memory'] for s in samples) / len(samples)
    
    print(f"Average CPU: {avg_cpu:.1f}%")
    print(f"Average Memory: {avg_memory:.1f}%")
    
    return samples
```

## Error Handling

### Retry Logic

```python
async def with_retry(coro_func, max_retries=3, delay=1.0):
    """Execute with exponential backoff retry."""
    
    for attempt in range(max_retries):
        try:
            return await coro_func()
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            
            wait_time = delay * (2 ** attempt)
            print(f"Attempt {attempt + 1} failed: {e}")
            print(f"Retrying in {wait_time} seconds...")
            await asyncio.sleep(wait_time)

# Usage
result = await with_retry(
    lambda: client.set_route({"goal_pose": goal_pose}),
    max_retries=3
)
```

### Timeout Handling

```python
async def with_timeout(coro, timeout_sec=10):
    """Execute with timeout."""
    try:
        async with asyncio.timeout(timeout_sec):
            return await coro
    except asyncio.TimeoutError:
        print(f"Operation timed out after {timeout_sec} seconds")
        return None

# Usage
result = await with_timeout(
    client.set_operation_mode("autonomous"),
    timeout_sec=15
)
```

### Error Recovery

```python
class AutowareController:
    def __init__(self, client):
        self.client = client
        self.error_count = 0
        self.max_errors = 3
    
    async def safe_execute(self, operation, *args, **kwargs):
        """Execute operation with error tracking."""
        try:
            result = await operation(*args, **kwargs)
            self.error_count = 0  # Reset on success
            return result
        except Exception as e:
            self.error_count += 1
            print(f"Error {self.error_count}/{self.max_errors}: {e}")
            
            if self.error_count >= self.max_errors:
                print("Max errors reached, stopping vehicle")
                await self.emergency_stop()
                raise
            
            return None
    
    async def emergency_stop(self):
        """Emergency stop procedure."""
        try:
            await self.client.set_operation_mode("stop")
        except:
            # Last resort
            await self.client.call_ros2_service(
                "/api/motion/emergency_stop",
                "std_srvs/srv/Trigger",
                {}
            )
```

## Testing

### Mock Client for Testing

```python
class MockAutowareMCPClient:
    """Mock client for testing."""
    
    async def connect(self):
        pass
    
    async def disconnect(self):
        pass
    
    async def health_check(self):
        return {"healthy": True, "ros2_nodes_count": 100}
    
    async def get_vehicle_state(self):
        return {
            "status": {"velocity": 2.5},
            "kinematics": {
                "pose": {
                    "position": {"x": 100, "y": 200, "z": 0}
                }
            }
        }
    
    # Add more mock methods as needed

# Use in tests
async def test_my_function():
    client = MockAutowareMCPClient()
    result = await my_function(client)
    assert result == expected_value
```

## Best Practices

1. **Always use context managers** for automatic cleanup
2. **Check operation results** before proceeding
3. **Implement proper error handling** with retries
4. **Monitor system health** continuously
5. **Use appropriate timeouts** for all operations
6. **Log important operations** for debugging
7. **Test with mock client** before real vehicle
8. **Handle connection loss** gracefully

## Advanced Usage

### Custom Tool Wrapper

```python
class AutowareExtendedClient(AutowareMCPClient):
    """Extended client with custom methods."""
    
    async def go_to_pose(self, pose, timeout=300):
        """High-level navigation to pose."""
        
        # Initialize if needed
        loc_state = await self.monitor_localization_state()
        if not loc_state['initialized']:
            await self.initialize_localization(pose)
            await asyncio.sleep(5)
        
        # Set route
        await self.set_route({"goal_pose": pose})
        
        # Wait for route
        for _ in range(10):
            route = await self.get_current_route()
            if route['state'] == 2:
                break
            await asyncio.sleep(1)
        else:
            raise Exception("Route not ready")
        
        # Go autonomous
        await self.set_operation_mode("autonomous")
        
        # Monitor until arrival
        start = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start < timeout:
            route = await self.get_current_route()
            if route['state'] == 3:
                await self.set_operation_mode("stop")
                return True
            await asyncio.sleep(1)
        
        return False
```

### Parallel Operations

```python
async def parallel_monitoring(client):
    """Run multiple operations in parallel."""
    
    # Gather multiple status checks
    results = await asyncio.gather(
        client.health_check(),
        client.get_system_status(),
        client.get_vehicle_state(),
        client.monitor_operation_mode(),
        client.monitor_diagnostics(),
        return_exceptions=True
    )
    
    health, system, vehicle, mode, diag = results
    
    # Process results
    status = {
        "healthy": health.get('healthy', False) if isinstance(health, dict) else False,
        "cpu": system.get('cpu', {}).get('percent', 0) if isinstance(system, dict) else 0,
        "speed": vehicle.get('status', {}).get('velocity', 0) if isinstance(vehicle, dict) else 0,
        "mode": mode.get('mode', 'unknown') if isinstance(mode, dict) else 'unknown',
        "errors": diag.get('errors', []) if isinstance(diag, dict) else []
    }
    
    return status
```

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| Connection timeout | Check server is running, increase timeout |
| Service not available | Verify ROS2 nodes are active |
| Invalid parameters | Check parameter types and ranges |
| Route planning fails | Clear existing route first |
| Mode change fails | Verify prerequisites met |

### Debug Mode

```python
import logging

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)

# Create client with debug
client = AutowareMCPClient(debug=True)

# Will print all MCP communication
```