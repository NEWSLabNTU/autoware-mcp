# CLAUDE.md - Autoware MCP Project Knowledge

## Project Overview
This project provides an MCP (Model Context Protocol) server for Autoware, enabling AI agents to control autonomous vehicles through a standardized interface.

## IMPORTANT: Working Examples Available
Two fully functional autonomous driving scripts are available in `examples/`:
- **autonomous_drive_mcp_tools.py** - Uses MCP server interface (recommended for AI agents)
- **autonomous_drive_ros_direct.py** - Uses direct ROS2 service calls
Both scripts include real-time monitoring until goal reached with speed tracking.

## Project Organization

### Build System: uv
The project uses uv for Python project management. Key commands:
- `uv sync --all-extras --dev` - Install dependencies and set up virtual environment
- `uv run autoware-mcp` - Run the MCP server
- `uv run pytest` - Run tests
- `uv run pytest -v` - Run tests with verbose output
- `uv run pytest --cov=autoware_mcp` - Run tests with coverage

### Project Structure
```
autoware-mcp/
├── src/autoware_mcp/
│   ├── server.py           # Main MCP server implementation
│   ├── ad_api_ros2.py      # Autoware AD API ROS2 interface
│   ├── launch_manager/     # Launch session management (NEW in Phase 4)
│   │   ├── session_manager.py  # Main session manager
│   │   ├── session.py          # Session state tracking
│   │   ├── process_tracker.py  # PID/PGID file management
│   │   ├── cleanup.py          # Cleanup and recovery
│   │   └── generator.py        # Launch file generation
│   └── tools/               # MCP tool implementations
├── examples/
│   ├── poses_config.yaml   # Validated initial and goal poses
│   ├── test_autonomous_drive_with_map.py  # Complete test example
│   ├── launch_management_demo.py  # Launch management demo (NEW)
│   └── run_planning_simulation.sh  # Simulation launcher script
├── tests/                   # Unit and integration tests
├── docs/                    # Documentation
│   ├── LAUNCH_SESSION_MANAGEMENT.md  # Launch management design
│   └── roadmap.md          # Development roadmap
└── pyproject.toml          # Rye project configuration
```

### Runtime Directory Structure (Created Automatically)
```
.autoware-mcp/
├── instance_<port>_<pid>/  # Unique per MCP server instance
│   ├── sessions/           # Active launch sessions
│   │   └── <session_id>/   # Per-session tracking files
│   │       ├── session.pid    # Main process PID
│   │       ├── session.pgid   # Process group ID
│   │       └── state.json     # Session state
│   └── mcp_server.pid      # MCP server PID
├── generated/              # AI-generated files
│   ├── launches/          # Generated launch files (versioned)
│   ├── nodes/            # Generated custom nodes
│   └── configs/          # Generated configurations
├── archived/              # Cleaned up sessions
└── logs/                  # Session logs
    └── session_<id>/
        ├── stdout.log
        └── stderr.log
```

## Critical Findings and Solutions

### 1. Route Setting Service Requirements
**Issue**: The SetRoute service was failing with "The planned route is empty"

**Root Cause**: Wrong service endpoint - should use SetRoutePoints instead of SetRoute

**Solution** (Fixed 2025-08-21): 
```python
# Fixed in src/autoware_mcp/ad_api_ros2.py
# Use /api/routing/set_route_points service instead
result = await self._call_service(
    "/api/routing/set_route_points",  # Changed from set_route
    "autoware_adapi_v1_msgs/srv/SetRoutePoints",
    {
        "header": {"frame_id": "map"},
        "goal": goal_pose,
        "waypoints": [],  # Empty waypoints means direct route to goal
        "option": option or {"allow_goal_modification": True},
    }
)
```
**Note**: The MCP server must be reconnected after this fix for changes to take effect

### 2. Localization Initialization
**SOLVED**: Vehicle position now correctly reads from `/localization/kinematic_state`

**Working Implementation**: Parse both pose and twist sections correctly:
```python
# See get_vehicle_state() in examples/autonomous_drive_mcp_tools.py
# Correctly parses position from pose.pose.position
# Correctly parses velocity from twist.twist.linear
```

**Validated Initial Pose** (for sample-map-planning):
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
```

**Topics for Localization**:
- `/initialpose` - For setting initial pose (PoseWithCovarianceStamped)
- `/localization/kinematic_state` - For reading current position
- `/tf` - Transform tree containing map->base_link transform

### 3. Service vs Topic Communication
**Finding**: Direct ROS2 service calls are more reliable than some MCP wrapper functions

**Recommended Approach**:
```python
# Use call_ros2_service for critical operations
mcp__autoware__call_ros2_service(
    service_name="/api/routing/set_route_points",
    service_type="autoware_adapi_v1_msgs/srv/SetRoutePoints",
    request={...}
)
```

### 4. Operation Mode Transitions
**Working Sequence**:
1. Start in STOP mode
2. Initialize localization with initial pose
3. Clear any existing route
4. Set route to goal
5. Verify route state is SET (state: 2)
6. Then transition to AUTONOMOUS mode
7. Monitor until vehicle reaches goal (state: 3 = ARRIVED)

**Route States**:
- 0 = UNKNOWN
- 1 = UNSET  
- 2 = SET (ready to drive)
- 3 = ARRIVED (reached destination)

## Simulation Management

### Starting Planning Simulation
```bash
./run_planning_simulation.sh start  # Start and wait for ready
./run_planning_simulation.sh stop   # Clean shutdown
./run_planning_simulation.sh status # Check if running
```

**Startup Phases**:
1. Phase 1: Wait for nodes (~10s)
2. Phase 2: Check critical topics
3. Phase 3: Verify map data loaded
4. Total startup: ~17-20 seconds

### Critical Topics to Monitor
- `/map/vector_map` - Must have data
- `/map/pointcloud_map` - Must have data  
- `/api/operation_mode/state` - Operation mode status
- `/api/routing/state` - Route status
- `/tf` and `/tf_static` - Transform tree

## NEW: Phase 4 - Launch Session Management (Implemented 2025-01-27)

### Overview
Complete launch session management system with PID/PGID tracking, preventing orphaned processes and enabling AI agents to generate and manage ROS2 launch files.

### Key Features
1. **Robust Process Management**
   - PID/PGID tracking for all launched processes
   - No orphaned processes after MCP crashes
   - Automatic cleanup on shutdown
   - Session recovery on restart

2. **Launch Control Tools**
   - `start_launch` - Start ROS2 launch files with tracking
   - `stop_launch` - Graceful shutdown with timeout
   - `pause_launch` - Suspend sessions (SIGSTOP)
   - `resume_launch` - Resume sessions (SIGCONT)
   - `restart_launch` - Stop and restart with same config
   - `list_launch_sessions` - View all active sessions
   - `get_session_status` - Detailed session information
   - `get_session_logs` - Retrieve stdout/stderr logs
   - `cleanup_orphans` - Clean up abandoned processes

3. **AI Development Tools**
   - `generate_launch_file` - Create launch files with templates
   - `generate_node_config` - Create YAML/JSON configs
   - `generate_custom_node` - Generate Python/C++ nodes
   - `validate_launch_file` - Check syntax and structure
   - `test_launch_file` - Dry-run validation
   - `get_launch_errors` - Retrieve error diagnostics
   - `list_generated_files` - Track AI-generated code

4. **Templates Available**
   - `perception_pipeline` - Lidar, camera, fusion setup
   - `planning_pipeline` - Planning components
   - `control_pipeline` - Control components
   - Generic template for custom configurations

### Usage Example
```python
# Generate a perception pipeline
await generate_launch_file(
    name="my_perception",
    components=["lidar_processing", "camera_detection"],
    template="perception_pipeline",
    parameters={"model_path": "./models/yolo.pt"}
)

# Start the launch file
result = await start_launch(
    launch_file=".autoware-mcp/generated/launches/my_perception_v1.launch.py",
    parameters={"use_sim_time": "true"}
)
session_id = result["session_id"]

# Monitor status
status = await get_session_status(session_id)
print(f"State: {status['state']}, PID: {status['main_pid']}")

# Stop when done
await stop_launch(session_id)
```

### Important Notes
- All generated files include version numbers (v1, v2, v3...)
- Sessions persist across MCP reconnections
- Process groups enable clean shutdown of entire launch trees
- Cleanup happens automatically on MCP shutdown
- Multiple MCP instances can run without conflicts

## Successfully Implemented Features

### 1. MCP Tools for Autonomous Driving (Fixed 2025-08-21)
**All MCP tools now working correctly**:
- `health_check` - System health monitoring
- `initialize_localization` - Set initial pose
- `set_route` - Plan route to goal (FIXED to use correct service)
- `get_current_route` - Monitor route state
- `set_operation_mode` - Change to autonomous/stop/etc
- `get_vehicle_state` - Get vehicle position and speed

### 2. Real-time Vehicle Monitoring
**Implementation**: Both example scripts now include continuous monitoring with:
- Position tracking from `/localization/kinematic_state`
- Speed calculation from twist.twist.linear velocities
- Distance to goal calculation
- Progress percentage
- Automatic detection of arrival (distance < 5m or route state = ARRIVED)
- Warning when vehicle is stuck (speed < 0.1 m/s for extended time)
- **Note**: Vehicle may show 0 speed temporarily at intersections/turns - this is normal

### 2. Goal Pose Capture from RViz
**How to capture new goal from RViz**:
```bash
ros2 topic echo /planning/mission_planning/echo_back_goal_pose --once
```
This captures the last goal pose clicked in RViz and can be saved to poses_config.yaml

### 3. Speed Reading Fixed
**Solution**: Correctly parse the kinematic state message structure:
- Position is in `pose.pose.position`
- Velocity is in `twist.twist.linear` (NOT `linear_velocity`)
- Speed = sqrt(vel_x² + vel_y²)

## Known Issues and Solutions

### Issue 1: Route Planning Failures  
**Symptom**: "The planned route is empty" error
**Solution**: Use `set_route_points` with empty segments array or add `segments: []` to set_route request

### Issue 2: Orphan Processes
**Symptom**: ROS2 processes remain after simulation stop
**Solution**: Use the provided script's stop command which properly cleans up

### Issue 3: EOF Error in Non-interactive Mode
**Symptom**: Script crashes with EOFError when run non-interactively
**Solution**: Wrap input() calls in try/except EOFError block

## Complete Autonomous Driving Procedure

### Prerequisites
1. Start planning simulation: `./run_planning_simulation.sh start`
2. Wait for full initialization (~17-20 seconds)
3. Verify all critical topics are available

### Autonomous Driving Sequence (PROVEN TO WORK)
1. **Load poses configuration** from `poses_config.yaml`
2. **Check system health** - Verify ROS2 topics are available
3. **Initialize localization** - Publish to `/initialpose` topic
4. **Wait 5 seconds** for localization to stabilize
5. **Clear existing route** - Call `/api/routing/clear_route` service
6. **Set route to goal** - Use `/api/routing/set_route_points` service
7. **Verify route state** - Check `/api/routing/state` for state: 2 (SET)
8. **Engage autonomous mode** - Call `/api/operation_mode/change_to_autonomous`
9. **Monitor progress** - Track position, speed, distance until arrival
10. **Detect arrival** - Route state: 3 (ARRIVED) or distance < 5 meters

### Expected Performance
- Typical journey time: 70-105 seconds for 46-meter route
- Speed range: 0.0 - 3.5 m/s (varies with road conditions)
- Vehicle will slow down in turns and complex sections
- Final positioning accuracy: within 1-3 meters of goal
- **Demo Results (2025-08-21)**: 74 seconds, max speed 3.05 m/s, final accuracy 1.5m

## Environment Variables
```bash
# Required for MCP server
export ROS_DOMAIN_ID=<your_domain_id>
export RMW_IMPLEMENTATION=<your_rmw_implementation>

# For simulation
export MAP_PATH="$HOME/autoware_map/sample-map-planning"
export VEHICLE_MODEL="sample_vehicle"
export SENSOR_MODEL="sample_sensor_kit"
export DISPLAY=":1"  # For headless environments
```

## Development Workflow

### Making Changes to MCP Server
1. Edit source files in `src/autoware_mcp/`
2. No rebuild needed (Python)
3. Restart MCP connection in Claude Code: `/mcp` command
4. Test changes immediately

### Running Tests
```bash
uv run pytest                    # Run all tests
uv run pytest tests/test_ad_api.py -v  # Run specific test
uv run pytest --cov=autoware_mcp  # Run tests with coverage
```

## Completed Improvements (Previously TODO)
✅ **Vehicle State Reading** - Now correctly reads from `/localization/kinematic_state`
✅ **Speed Monitoring** - Accurately tracks vehicle speed from twist data
✅ **Goal Detection** - Monitors until vehicle reaches destination
✅ **Progress Tracking** - Shows real-time progress percentage
✅ **Launch Session Management** - Complete Phase 4 implementation with PID/PGID tracking
✅ **Process Cleanup** - No orphaned processes after MCP crashes
✅ **Launch Generation** - AI can generate and test launch files iteratively
✅ **Session Recovery** - Sessions persist across MCP reconnections

## Future Improvements
1. **Add Localization Validation** - Wait for and verify successful localization before allowing route setting
2. **Improve Error Recovery** - Add automatic retry logic for transient failures
3. **Enhanced Diagnostics** - More detailed error messages when services fail
4. **Traffic Light Integration** - Handle traffic signals in autonomous mode
5. **Obstacle Detection** - Monitor and report obstacles in vehicle path

## Quick Reference

### Core MCP Tools
- `health_check` - System health status
- `initialize_localization` - Set initial pose
- `set_route` / `set_route_points` - Plan route
- `set_operation_mode` - Change driving mode
- `get_vehicle_state` - Get vehicle info
- `call_ros2_service` - Direct service calls
- `publish_to_topic` - Publish to any topic

### Launch Management Tools (NEW)
- `start_launch` - Start ROS2 launch files
- `stop_launch` - Stop launch sessions
- `pause_launch` / `resume_launch` - Suspend/resume
- `restart_launch` - Restart sessions
- `list_launch_sessions` - View active sessions
- `get_session_status` - Session details
- `get_session_logs` - Retrieve logs
- `cleanup_orphans` - Clean abandoned processes

### AI Development Tools (NEW)
- `generate_launch_file` - Create launch files
- `generate_node_config` - Create configs
- `generate_custom_node` - Generate nodes
- `validate_launch_file` - Check syntax
- `test_launch_file` - Dry-run test
- `list_generated_files` - Track generated code

### Common ROS2 Commands
```bash
ros2 node list              # List active nodes
ros2 topic list             # List topics
ros2 service list           # List services
ros2 topic echo /tf        # Monitor transforms
ros2 service call /api/routing/clear_route  # Clear route
```

## Contact and Resources
- Autoware Documentation: https://autowarefoundation.github.io/autoware-documentation/
- Sample Map Location: `~/autoware_map/sample-map-planning/`
- Log Files: `/tmp/planning_simulation.log`

## Important Notes for Future Sessions

1. **Always check if simulation is already running** before restarting - previous runs may still be active
2. **Let autonomous driving complete** - Vehicle continues to goal even after script exits
3. **Use the example scripts** - Both `autonomous_drive_mcp_tools.py` and `autonomous_drive_ros_direct.py` are fully functional
4. **Monitor vehicle progress** - Real-time monitoring is implemented and working
5. **Capture RViz goals** - Use `/planning/mission_planning/echo_back_goal_pose` to get clicked goals

---
*Last Updated: 2025-01-27*
*Status: PHASE 4 COMPLETE - Launch session management fully implemented*

### Recent Updates
- **2025-01-27**: Phase 4 Launch Session Management completed
  - Full PID/PGID tracking prevents orphaned processes
  - AI-friendly launch file generation with versioning
  - Session recovery across MCP restarts
  - 18 new MCP tools for launch management
  - 32 comprehensive tests all passing

- **2025-08-21**: Core functionality verified
  - MCP tools fixed with successful autonomous driving demos
  - set_route now uses /api/routing/set_route_points service

*Note: Reconnect MCP in Claude Code after code changes for updates to take effect*