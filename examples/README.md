# Autoware MCP Examples

This directory contains example scripts demonstrating how to use the Autoware MCP server for autonomous driving tasks.

## Two Main Approaches

### 1. Direct ROS2 Service Calls (`autonomous_drive_ros_direct.py`)
Uses subprocess to call ROS2 services directly without MCP server:
- Direct control over service calls
- No MCP server dependency
- Good for debugging and understanding the underlying API
- Useful when MCP server is not available

```bash
# Ensure planning simulation is running first
./run_planning_simulation.sh start

# Run the direct ROS2 example
python3 autonomous_drive_ros_direct.py
```

### 2. MCP Tools Approach (`autonomous_drive_mcp_tools.py`)
Uses Autoware MCP server as an abstraction layer:
- Cleaner async/await interface
- Better error handling and state management
- Recommended for production use
- Provides higher-level abstractions

```bash
# Ensure planning simulation is running first
./run_planning_simulation.sh start

# MCP server will be started automatically by the script
python3 autonomous_drive_mcp_tools.py
```

## Configuration

Both scripts load poses from `poses_config.yaml`:

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
    x: 3711.556396484375
    y: 73698.53125
    z: 19.349462309662066
  orientation:
    x: 0.0
    y: 0.0
    z: 0.8505915923362147
    w: 0.5258269135817821
```

These poses are validated for the `sample-map-planning` map.

## Prerequisites

Before running these examples, ensure:

1. **Environment is sourced**: Source your ROS2 and Autoware workspace
2. **Start planning simulation**: `./run_planning_simulation.sh start`
3. **RViz is open**: The simulation script starts RViz automatically
4. **For MCP approach only**: Python environment with MCP installed (`rye sync`)

## Workflow Overview

Both scripts follow this workflow:

1. **Load Configuration** - Read poses from `poses_config.yaml`
2. **Check System** - Verify Autoware is ready
3. **Initialize Localization** - Set initial vehicle pose
4. **Clear Route** - Remove any existing route
5. **Set Route to Goal** - Plan path to destination
6. **Enable Autonomous Mode** - Start autonomous driving
7. **Monitor Progress** - Track vehicle movement
8. **Stop Vehicle** - Return to stop mode (optional)

## Key Differences

| Aspect         | Direct ROS2             | MCP Tools                |
|----------------|-------------------------|--------------------------|
| Dependencies   | Only ROS2               | MCP server + client libs |
| Error Handling | Basic subprocess checks | Structured exceptions    |
| Async Support  | No (uses subprocess)    | Yes (native async/await) |
| Type Safety    | String-based            | Python dict/object based |
| Debugging      | See raw ROS2 output     | Parsed responses         |
| Performance    | Subprocess overhead     | Direct Python calls      |

## Supporting Files

- `poses_config.yaml` - Validated initial and goal poses for sample-map-planning
- `run_planning_simulation.sh` - Helper script to manage the planning simulation
- `test_results.md` - Documentation of test findings and known issues

## Troubleshooting

### Vehicle Not Moving in Simulation
1. Check RViz - the vehicle should be moving there
2. The MCP `get_vehicle_state` may show position as (0,0,0) due to implementation issue
3. This is a known issue - vehicle IS moving correctly in the simulation

### Route Planning Fails
1. Ensure localization is initialized first
2. Goal must be reachable on the road network
3. Try using `set_route_points` instead of `set_route`
4. Check the logs in `/tmp/planning_simulation.log`

### MCP Server Connection Issues
1. Ensure ROS2 environment is sourced
2. Check RMW_IMPLEMENTATION and ROS_DOMAIN_ID are set
3. Try reconnecting with `/mcp` command in Claude Code
4. Verify with `rye run autoware-mcp` manually

## Testing Workflow

For comprehensive testing:

```bash
# 1. Start simulation
./run_planning_simulation.sh start

# 2. Test with direct ROS2 (no MCP needed)
python3 autonomous_drive_ros_direct.py

# 3. Stop and restart for clean state
./run_planning_simulation.sh restart

# 4. Test with MCP tools
python3 autonomous_drive_mcp_tools.py

# 5. Clean up when done
./run_planning_simulation.sh stop
```

## Integration with AI Agents

These examples can be adapted for AI agent control:

```python
# AI agent can use either approach

# Direct ROS2 approach - good for simple integrations
subprocess.run(["python3", "autonomous_drive_ros_direct.py"])

# MCP approach - better for complex agent logic
async def agent_navigate():
    client = AutowareMCPClient()
    await client.connect()
    await client.run_autonomous_sequence()
    await client.disconnect()
```

## Additional Resources

- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)
- [Autoware AD API Reference](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/)
- [MCP Protocol Documentation](https://modelcontextprotocol.io/)
- [Project CLAUDE.md](../CLAUDE.md) - Implementation notes and findings
