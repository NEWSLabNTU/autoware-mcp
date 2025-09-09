# Common Scenarios

## Scenario 1: Autonomous Navigation

### Goal
Navigate from point A to B autonomously.

### Steps
```python
# 1. Start simulation
session = await start_launch("planning_simulator.launch.xml", {
    "map_path": "~/autoware_map/sample-map-planning"
})

# 2. Initialize localization
await initialize_localization(initial_pose)

# 3. Set route to goal
await set_route(goal_pose)

# 4. Verify route is set
route = await get_current_route()
assert route["state"] == "SET"

# 5. Start autonomous driving
await set_operation_mode("autonomous")

# 6. Monitor progress
while True:
    state = await get_vehicle_state()
    if await get_current_route()["state"] == "ARRIVED":
        break
```

## Scenario 2: Rosbag Replay

### Goal
Replay sensor data for testing perception.

### Steps
```python
# 1. Create launch file for rosbag replay
launch_xml = '''
<launch>
  <include file="$(find-pkg-share autoware_launch)/launch/logging_simulator.launch.xml">
    <arg name="map_path" value="~/autoware_map/sample-map-rosbag"/>
  </include>
  <executable cmd="ros2 bag play ~/autoware_map/sample-rosbag --rate 0.2 --clock --loop"/>
</launch>
'''

# 2. Start replay
session = await start_launch("rosbag_replay.launch.xml")

# 3. Monitor perception
while True:
    objects = await get_detected_objects()
    print(f"Detected: {len(objects['objects'])} objects")
    await asyncio.sleep(1)
```

## Scenario 3: Emergency Stop

### Goal
Safely stop vehicle in emergency situation.

### Steps
```python
# 1. Detect emergency condition
objects = await get_detected_objects()
obstacle_ahead = check_collision_risk(objects)

# 2. Request emergency stop
if obstacle_ahead:
    await request_mrm("emergency_stop", "Obstacle detected")
    
# 3. Monitor MRM state
mrm_state = await monitor_mrm_state()
print(f"MRM active: {mrm_state['active']}")

# 4. Resume when safe
if is_safe():
    await set_operation_mode("autonomous")
```

## Scenario 4: Multi-Session Management

### Goal
Run multiple launch files concurrently.

### Steps
```python
# 1. Start planning simulation
planning = await start_launch("planning_simulator.launch.xml")

# 2. Start visualization
rviz = await start_launch("rviz2.launch.py", {
    "rviz_config": "autoware.rviz"
})

# 3. Start data recording
recording = await start_launch("rosbag_record.launch.py", {
    "topics": ["/tf", "/perception/object_recognition/objects"]
})

# 4. List all sessions
sessions = await list_launch_sessions()
for s in sessions:
    print(f"{s['session_id']}: {s['state']}")

# 5. Clean shutdown
for session_id in [planning, rviz, recording]:
    await stop_launch(session_id)
```

## Scenario 5: Perception Analysis

### Goal
Analyze scene for AI decision making.

### Steps
```python
# 1. Capture multi-modal perception
scene = await analyze_driving_scene()

# 2. Process camera images
for camera in ["front", "rear", "left", "right"]:
    img = await capture_camera_view(camera)
    # AI processes img["image_path"]

# 3. Visualize LiDAR
lidar_bev = await visualize_lidar_scene("bev")
lidar_3d = await visualize_lidar_scene("front")

# 4. Get object list
vehicles = await get_detected_objects("vehicle")
pedestrians = await get_detected_objects("pedestrian")

# 5. Make driving decision
decision = analyze_scene_safety(scene, vehicles, pedestrians)
if decision == "proceed":
    await set_operation_mode("autonomous")
```

## Scenario 6: Route Modification

### Goal
Change route while driving.

### Steps
```python
# 1. Start driving to initial goal
await set_route(goal_a)
await set_operation_mode("autonomous")

# 2. Detect need for reroute
traffic = await check_traffic_ahead()
if traffic["congested"]:
    # 3. Clear current route
    await call_ros2_service(
        "/api/routing/clear_route",
        "autoware_adapi_v1_msgs/srv/ClearRoute",
        {}
    )
    
    # 4. Set new route via waypoints
    await set_route_points([waypoint_b, waypoint_c, goal_b])
    
    # 5. Continue driving
    await set_operation_mode("autonomous")
```

## Scenario 7: Manual Override

### Goal
Take manual control during autonomous driving.

### Steps
```python
# 1. Driving autonomously
await set_operation_mode("autonomous")

# 2. Detect manual intervention needed
situation = await analyze_complex_scenario()
if situation["requires_human"]:
    # 3. Switch to manual mode
    await set_operation_mode("local")
    
    # 4. Send manual commands
    await send_velocity_command(5.0)  # 5 m/s
    await send_steering_command(0.1)  # Slight right
    
    # 5. Return to autonomous when ready
    if situation["resolved"]:
        await set_operation_mode("autonomous")
```

## Error Handling Patterns

### Retry Pattern
```python
async def retry_operation(func, max_retries=3):
    for i in range(max_retries):
        try:
            result = await func()
            if result["success"]:
                return result
        except Exception as e:
            print(f"Attempt {i+1} failed: {e}")
        await asyncio.sleep(2 ** i)  # Exponential backoff
    raise Exception("Max retries exceeded")
```

### Graceful Degradation
```python
async def safe_navigate():
    try:
        # Try autonomous
        await set_operation_mode("autonomous")
    except:
        try:
            # Fall back to MRM
            await request_mrm("comfortable_stop")
        except:
            # Emergency stop as last resort
            await send_velocity_command(0)
            await send_pedals_command(throttle=0, brake=1.0)
```