# Autonomous Driving Examples

Complete examples and patterns for autonomous driving scenarios with Autoware.

## Example 1: Simple Point-to-Point Navigation

The most basic autonomous driving scenario - navigate from point A to point B.

### Prerequisites

1. Planning simulation running:
```bash
cd ~/autoware-mcp
./scripts/run_planning_simulation.sh start
```

2. MCP server active:
```bash
uv run autoware-mcp
```

3. Poses configuration (`examples/poses_config.yaml`):
```yaml
initial_pose:
  position:
    x: 3752.342041015625
    y: 73736.09375
    z: 19.3404
  orientation:
    x: -0.0008153484821679761
    y: -0.00024282468558667022
    z: -0.9583998316115443
    w: 0.2854278175125686

goal_pose:
  position:
    x: 3798.71
    y: 73775.45
    z: 0
  orientation:
    x: 0
    y: 0
    z: 0.862048
    w: 0.506823
```

### Complete Implementation

```python
import asyncio
import yaml
from pathlib import Path
from autoware_mcp.client import AutowareMCPClient

async def simple_autonomous_drive():
    """Simple point-to-point autonomous navigation."""
    
    # Initialize client
    client = AutowareMCPClient()
    await client.connect()
    
    # Load poses
    config_path = Path("examples/poses_config.yaml")
    with open(config_path, "r") as f:
        poses = yaml.safe_load(f)
    
    print("Starting autonomous drive sequence...")
    
    # 1. Check system health
    health = await client.health_check()
    if not health['healthy']:
        print("System not healthy!")
        return False
    print(f"✓ System healthy - {health['ros2_nodes_count']} nodes active")
    
    # 2. Initialize localization
    result = await client.initialize_localization(poses['initial_pose'])
    if not result['success']:
        print("Failed to initialize localization")
        return False
    print("✓ Localization initialized")
    
    # Wait for localization to stabilize
    await asyncio.sleep(5)
    
    # 3. Clear any existing route
    await client.call_ros2_service(
        "/api/routing/clear_route",
        "std_srvs/srv/Trigger",
        {}
    )
    print("✓ Route cleared")
    
    # 4. Set route to goal
    route_result = await client.set_route({"goal_pose": poses['goal_pose']})
    if not route_result['success']:
        print("Failed to set route")
        return False
    print("✓ Route set to goal")
    
    # Wait for route processing
    await asyncio.sleep(3)
    
    # 5. Verify route state
    route_state = await client.get_current_route()
    if route_state['state'] != 2:  # 2 = SET
        print(f"Route not ready: state={route_state['state']}")
        return False
    print("✓ Route verified")
    
    # 6. Engage autonomous mode
    mode_result = await client.set_operation_mode("autonomous")
    if not mode_result['success']:
        print("Failed to engage autonomous mode")
        return False
    print("✓ AUTONOMOUS MODE ENGAGED")
    
    # 7. Monitor progress to goal
    print("\nMonitoring journey...")
    print("-" * 40)
    
    start_time = asyncio.get_event_loop().time()
    while True:
        # Get vehicle state
        state = await client.get_vehicle_state()
        position = state['kinematics']['pose']['position']
        speed = state['status']['velocity']
        
        # Calculate distance to goal
        goal = poses['goal_pose']['position']
        dx = goal['x'] - position['x']
        dy = goal['y'] - position['y']
        distance = (dx**2 + dy**2) ** 0.5
        
        # Check route state
        route = await client.get_current_route()
        
        # Display progress
        elapsed = asyncio.get_event_loop().time() - start_time
        print(
            f"[{elapsed:5.1f}s] "
            f"Speed: {speed:4.1f} m/s | "
            f"Distance: {distance:5.1f}m | "
            f"State: {route['state']}"
        )
        
        # Check if arrived
        if route['state'] == 3 or distance < 5.0:  # 3 = ARRIVED
            print("-" * 40)
            print(f"\n✓ GOAL REACHED in {elapsed:.1f} seconds!")
            print(f"Final distance: {distance:.1f}m")
            break
        
        # Check for stuck vehicle
        if speed < 0.1 and elapsed > 30 and distance > 10:
            print("\n⚠ Vehicle appears stuck")
        
        await asyncio.sleep(2)
    
    # 8. Stop vehicle
    await client.set_operation_mode("stop")
    print("✓ Vehicle stopped")
    
    return True

# Run the example
asyncio.run(simple_autonomous_drive())
```

### Expected Output

```
Starting autonomous drive sequence...
✓ System healthy - 142 nodes active
✓ Localization initialized
✓ Route cleared
✓ Route set to goal
✓ Route verified
✓ AUTONOMOUS MODE ENGAGED

Monitoring journey...
----------------------------------------
[ 2.1s] Speed:  0.2 m/s | Distance: 45.8m | State: 2
[ 4.2s] Speed:  1.5 m/s | Distance: 43.2m | State: 2
[ 6.3s] Speed:  2.8 m/s | Distance: 38.5m | State: 2
...
[72.4s] Speed:  1.2 m/s | Distance:  3.2m | State: 3
----------------------------------------

✓ GOAL REACHED in 72.4 seconds!
Final distance: 3.2m
✓ Vehicle stopped
```

### Key Points

- **Localization stabilization**: Always wait 5 seconds after initialization
- **Route verification**: Check state = 2 (SET) before engaging autonomous mode
- **Distance threshold**: Consider arrived when < 5 meters from goal
- **Speed monitoring**: Detect stuck vehicle (speed < 0.1 m/s for > 30s)
- **Typical performance**: 70-105 seconds for 46-meter journey

## Example 2: Multi-Waypoint Mission

Navigate through multiple waypoints to reach the final destination.

```python
async def multi_waypoint_mission():
    """Navigate through multiple waypoints."""
    
    client = AutowareMCPClient()
    await client.connect()
    
    # Define waypoints
    waypoints = [
        {  # Waypoint 1: Turn corner
            "position": {"x": 3770.5, "y": 73750.2, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": -0.7, "w": 0.7}
        },
        {  # Waypoint 2: Midway
            "position": {"x": 3785.3, "y": 73762.8, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0.0, "w": 1.0}
        },
        {  # Final goal
            "position": {"x": 3798.71, "y": 73775.45, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0.862, "w": 0.507}
        }
    ]
    
    # Initialize localization
    await client.initialize_localization(initial_pose)
    await asyncio.sleep(5)
    
    # Clear route
    await client.call_ros2_service(
        "/api/routing/clear_route",
        "std_srvs/srv/Trigger",
        {}
    )
    
    # Set route with waypoints
    result = await client.set_route_points(waypoints)
    if not result['success']:
        print("Failed to set waypoint route")
        return False
    
    print(f"✓ Route set with {len(waypoints)} waypoints")
    
    # Engage autonomous mode
    await client.set_operation_mode("autonomous")
    
    # Monitor progress through waypoints
    current_waypoint = 0
    while current_waypoint < len(waypoints):
        state = await client.get_vehicle_state()
        position = state['kinematics']['pose']['position']
        
        # Check distance to current waypoint
        wp = waypoints[current_waypoint]['position']
        distance = ((wp['x'] - position['x'])**2 + 
                   (wp['y'] - position['y'])**2) ** 0.5
        
        print(f"Waypoint {current_waypoint + 1}/{len(waypoints)}: {distance:.1f}m")
        
        # Check if reached waypoint
        if distance < 5.0:
            print(f"✓ Reached waypoint {current_waypoint + 1}")
            current_waypoint += 1
        
        # Check final arrival
        route = await client.get_current_route()
        if route['state'] == 3:  # ARRIVED
            print("✓ Mission complete!")
            break
        
        await asyncio.sleep(2)
    
    await client.set_operation_mode("stop")
    return True
```

## Example 3: Dynamic Re-routing

Handle route changes and obstacles during navigation.

```python
async def dynamic_rerouting():
    """Demonstrate dynamic route modification."""
    
    client = AutowareMCPClient()
    await client.connect()
    
    # Start normal navigation
    await client.initialize_localization(initial_pose)
    await asyncio.sleep(5)
    
    original_goal = {
        "position": {"x": 3798.71, "y": 73775.45, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0.862, "w": 0.507}
    }
    
    await client.set_route({"goal_pose": original_goal})
    await client.set_operation_mode("autonomous")
    
    print("Driving to original goal...")
    
    # Monitor for 20 seconds
    for _ in range(10):
        state = await client.get_vehicle_state()
        print(f"Speed: {state['status']['velocity']:.1f} m/s")
        await asyncio.sleep(2)
    
    # Simulate obstacle detection - change route
    print("\n⚠ Obstacle detected! Re-routing...")
    
    # Stop temporarily
    await client.set_operation_mode("stop")
    await asyncio.sleep(2)
    
    # Clear current route
    await client.call_ros2_service(
        "/api/routing/clear_route",
        "std_srvs/srv/Trigger",
        {}
    )
    
    # Set new goal (alternate route)
    alternate_goal = {
        "position": {"x": 3780.0, "y": 73760.0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0.5, "w": 0.866}
    }
    
    await client.set_route({"goal_pose": alternate_goal})
    print("✓ New route set")
    
    # Resume autonomous driving
    await client.set_operation_mode("autonomous")
    print("✓ Resumed with new route")
    
    # Monitor until new goal
    while True:
        route = await client.get_current_route()
        if route['state'] == 3:
            print("✓ Reached alternate destination")
            break
        await asyncio.sleep(2)
    
    await client.set_operation_mode("stop")
    return True
```

## Example 4: Emergency Stop and Recovery

Handle emergency situations with proper stop and recovery.

```python
async def emergency_stop_recovery():
    """Demonstrate emergency stop and recovery."""
    
    client = AutowareMCPClient()
    await client.connect()
    
    # Start normal operation
    await client.initialize_localization(initial_pose)
    await asyncio.sleep(5)
    await client.set_route({"goal_pose": goal_pose})
    await client.set_operation_mode("autonomous")
    
    print("Driving normally...")
    
    # Monitor for 10 seconds
    for i in range(5):
        state = await client.get_vehicle_state()
        print(f"Speed: {state['status']['velocity']:.1f} m/s")
        await asyncio.sleep(2)
    
    # EMERGENCY STOP
    print("\n🚨 EMERGENCY STOP!")
    
    # Multiple redundant stop commands
    await asyncio.gather(
        client.set_operation_mode("stop"),
        client.send_velocity_command(velocity=0),
        client.send_pedals_command(throttle=0, brake=1),
        client.request_mrm(
            behavior="emergency_stop",
            reason="Manual emergency trigger"
        )
    )
    
    print("✓ Emergency stop executed")
    await asyncio.sleep(5)
    
    # Recovery sequence
    print("\nStarting recovery sequence...")
    
    # 1. Clear MRM state
    print("1. Clearing emergency state...")
    await client.call_ros2_service(
        "/api/motion/clear_emergency",
        "std_srvs/srv/Trigger",
        {}
    )
    
    # 2. Reset control
    print("2. Resetting control...")
    await client.send_pedals_command(throttle=0, brake=0)
    
    # 3. Verify route still valid
    print("3. Checking route...")
    route = await client.get_current_route()
    if route['state'] != 2:
        print("   Re-setting route...")
        await client.set_route({"goal_pose": goal_pose})
    
    # 4. Resume autonomous mode
    print("4. Resuming autonomous mode...")
    await client.set_operation_mode("autonomous")
    
    print("✓ Recovery complete - resuming journey")
    
    # Continue monitoring
    while True:
        route = await client.get_current_route()
        if route['state'] == 3:
            print("✓ Reached destination after recovery")
            break
        await asyncio.sleep(2)
    
    return True
```

## Example 5: Continuous Monitoring with Diagnostics

Comprehensive monitoring with health checks and diagnostics.

```python
async def monitored_autonomous_drive():
    """Autonomous drive with comprehensive monitoring."""
    
    client = AutowareMCPClient()
    await client.connect()
    
    # Initialize
    await client.initialize_localization(initial_pose)
    await asyncio.sleep(5)
    await client.set_route({"goal_pose": goal_pose})
    
    # Start monitoring tasks
    async def monitor_diagnostics():
        """Monitor system diagnostics."""
        while True:
            diag = await client.monitor_diagnostics()
            if not diag['healthy']:
                print(f"⚠️ System issue: {diag['errors']}")
            await asyncio.sleep(5)
    
    async def monitor_vehicle():
        """Monitor vehicle state."""
        while True:
            state = await client.get_vehicle_state()
            speed = state['status']['velocity']
            gear = state['status']['gear']
            
            # Check for anomalies
            if gear == "REVERSE" and speed > 0:
                print("⚠️ Unexpected reverse gear!")
            
            if speed > 10.0:  # Speed limit
                print(f"⚠️ Speed limit exceeded: {speed:.1f} m/s")
            
            await asyncio.sleep(1)
    
    async def monitor_route():
        """Monitor route progress."""
        while True:
            route = await client.get_current_route()
            
            if route['state'] == 1:  # UNSET
                print("⚠️ Route lost! Re-planning...")
                await client.set_route({"goal_pose": goal_pose})
            
            elif route['state'] == 3:  # ARRIVED
                print("✓ Destination reached")
                return True
            
            await asyncio.sleep(2)
    
    # Start autonomous mode
    await client.set_operation_mode("autonomous")
    
    # Run all monitors concurrently
    try:
        await asyncio.gather(
            monitor_diagnostics(),
            monitor_vehicle(),
            monitor_route()
        )
    except asyncio.CancelledError:
        pass
    finally:
        await client.set_operation_mode("stop")
    
    return True
```

## Best Practices

### 1. Always Initialize Properly
```python
# Standard initialization sequence
await client.initialize_localization(initial_pose)
await asyncio.sleep(5)  # Critical: wait for stabilization
await client.call_ros2_service("/api/routing/clear_route", "std_srvs/srv/Trigger", {})
```

### 2. Verify State Before Actions
```python
# Check route before engaging autonomous
route = await client.get_current_route()
if route['state'] != 2:  # Not SET
    print("Route not ready")
    return False
```

### 3. Implement Proper Error Handling
```python
try:
    result = await client.set_operation_mode("autonomous")
    if not result['success']:
        raise Exception(f"Mode change failed: {result.get('message')}")
except Exception as e:
    await client.set_operation_mode("stop")  # Safety first
    raise
```

### 4. Monitor Multiple Conditions
```python
# Check both route state and distance
if route['state'] == 3 or distance < 5.0:
    print("Goal reached")
```

### 5. Use Timeouts
```python
start_time = asyncio.get_event_loop().time()
while asyncio.get_event_loop().time() - start_time < 300:  # 5-minute timeout
    # ... monitoring code ...
```

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Vehicle not moving | Localization not initialized | Wait 5 seconds after initialization |
| Route planning fails | Previous route active | Clear route before setting new one |
| Stuck at intersection | Traffic light/obstacle | Monitor speed, implement timeout |
| Mode change fails | Prerequisites not met | Verify localization and route state |
| Erratic movement | Control parameters | Reduce speed limits, tune parameters |

## Performance Metrics

- **Typical journey time**: 70-105 seconds for 46m
- **Maximum speed**: 3.5 m/s (planning simulation)
- **Positioning accuracy**: ±1-3 meters
- **Localization convergence**: 3-5 seconds
- **Route planning time**: 1-3 seconds

## Next Steps

- Explore [Vehicle Control](./vehicle-control.md) for manual operations
- Learn [Launch Management](./launch-management.md) for complex scenarios
- Study [Safety Systems](../advanced/mrm-safety.md) for robust operations