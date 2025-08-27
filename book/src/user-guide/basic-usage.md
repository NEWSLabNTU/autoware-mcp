# Basic Usage

This guide covers the fundamental operations you'll use most frequently with the Autoware MCP Server.

## Starting the System

### 1. Launch Autoware Simulation

```bash
# Start planning simulation
cd ~/autoware-mcp
./scripts/run_planning_simulation.sh start
```

### 2. Start MCP Server

```bash
# In a new terminal
cd ~/autoware-mcp
uv run autoware-mcp
```

### 3. Connect Your AI Agent

For Claude Desktop, it auto-connects. For custom agents:

```python
from autoware_mcp.client import AutowareMCPClient

client = AutowareMCPClient()
await client.connect()
```

## Basic Operations

### System Health Check

Always start by verifying system health:

```python
health = await client.health_check()
print(f"System healthy: {health['healthy']}")
print(f"ROS2 nodes: {health['ros2_nodes_count']}")
print(f"Active topics: {health['ros2_topics_count']}")
```

### Vehicle Initialization

Initialize the vehicle's position on the map:

```python
# Standard initial pose for sample map
initial_pose = {
    "pose": {
        "position": {"x": 3752.34, "y": 73736.09, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": -0.958, "w": 0.285}
    }
}

result = await client.initialize_localization(initial_pose)
print(f"Localization initialized: {result['success']}")

# Wait for localization to converge
await asyncio.sleep(5)
```

### Route Planning

Set a destination and plan a route:

```python
# Define goal position
goal_pose = {
    "position": {"x": 3798.71, "y": 73775.45, "z": 0},
    "orientation": {"x": 0, "y": 0, "z": 0.862, "w": 0.506}
}

# Set the route
route_result = await client.set_route({"goal_pose": goal_pose})
print(f"Route planned: {route_result['success']}")

# Check route status
route_state = await client.get_current_route()
print(f"Route state: {route_state['state']}")  # Should be 2 (SET)
```

### Start Autonomous Driving

Enable autonomous mode to begin driving:

```python
# Change to autonomous mode
mode_result = await client.set_operation_mode("autonomous")
print(f"Autonomous mode enabled: {mode_result['success']}")

# Monitor the journey
while True:
    state = await client.get_vehicle_state()
    speed = state['status']['velocity']
    position = state['kinematics']['pose']['position']
    
    print(f"Speed: {speed:.2f} m/s")
    print(f"Position: ({position['x']:.1f}, {position['y']:.1f})")
    
    # Check if arrived
    route = await client.get_current_route()
    if route['state'] == 3:  # ARRIVED
        print("Destination reached!")
        break
    
    await asyncio.sleep(1)
```

### Emergency Stop

Always be ready to stop the vehicle:

```python
# Immediate stop
await client.set_operation_mode("stop")
print("Vehicle stopped")
```

## Working with Launch Files

### Starting a Launch Session

```python
# Launch a set of nodes
result = await client.start_launch("my_perception.launch.py")
session_id = result["session_id"]
print(f"Launch session started: {session_id}")

# Monitor session
status = await client.get_session_status(session_id)
print(f"Session state: {status['state']}")
print(f"Active nodes: {status['node_count']}")
```

### Managing Sessions

```python
# List all sessions
sessions = await client.list_launch_sessions()
for session in sessions:
    print(f"Session {session['session_id']}: {session['state']}")

# Pause a session
await client.pause_launch(session_id)

# Resume a session
await client.resume_launch(session_id)

# Stop a session
await client.stop_launch(session_id)
```

## Monitoring Operations

### Real-time Vehicle Monitoring

```python
async def monitor_vehicle():
    while True:
        state = await client.get_vehicle_state()
        
        # Extract key information
        speed = state['status']['velocity']
        gear = state['status']['gear']
        position = state['kinematics']['pose']['position']
        
        # Display status
        print(f"\rSpeed: {speed:5.2f} m/s | "
              f"Gear: {gear:6s} | "
              f"Pos: ({position['x']:7.1f}, {position['y']:7.1f})", 
              end='')
        
        await asyncio.sleep(0.1)
```

### Diagnostics Monitoring

```python
async def monitor_diagnostics():
    while True:
        diag = await client.monitor_diagnostics()
        
        if not diag['healthy']:
            print(f"⚠️ System issues detected:")
            for error in diag['errors']:
                print(f"  - {error['component']}: {error['message']}")
        
        await asyncio.sleep(5)
```

## Common Patterns

### Safe Initialization Pattern

```python
async def safe_initialize():
    try:
        # Check health first
        health = await client.health_check()
        if not health['healthy']:
            raise Exception("System not healthy")
        
        # Initialize localization
        await client.initialize_localization(initial_pose)
        await asyncio.sleep(5)
        
        # Clear any existing route
        await client.call_ros2_service(
            "/api/routing/clear_route",
            "std_srvs/srv/Trigger",
            {}
        )
        
        return True
    except Exception as e:
        print(f"Initialization failed: {e}")
        return False
```

### Route with Retry Pattern

```python
async def set_route_with_retry(goal_pose, max_retries=3):
    for attempt in range(max_retries):
        try:
            result = await client.set_route({"goal_pose": goal_pose})
            if result['success']:
                return True
            
            print(f"Route attempt {attempt + 1} failed")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"Route error: {e}")
    
    return False
```

### Monitoring with Timeout Pattern

```python
async def wait_for_arrival(timeout=300):
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        route = await client.get_current_route()
        
        if route['state'] == 3:  # ARRIVED
            return True
        
        if route['state'] == 1:  # UNSET
            print("Route lost!")
            return False
        
        await asyncio.sleep(1)
    
    print("Timeout waiting for arrival")
    return False
```

## Tips and Best Practices

1. **Always initialize localization** before any navigation
2. **Check operation mode** before sending commands
3. **Monitor diagnostics** continuously for early warning
4. **Use try-except blocks** for all MCP calls
5. **Implement timeouts** for long operations
6. **Log all operations** for debugging
7. **Test in simulation** before real vehicle

## Next Steps

- Explore [Autonomous Driving Examples](./autonomous-driving.md)
- Learn about [Launch Management](./launch-management.md)
- Master [Vehicle Control](./vehicle-control.md)
- Study [Advanced Topics](../advanced/localization.md)