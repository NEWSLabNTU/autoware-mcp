# Troubleshooting

This guide helps you diagnose and fix common issues with the Autoware MCP Server.

## Common Issues

### Installation Issues

#### uv command not found

**Problem**: After installation, `uv` command is not available.

**Solution**:
```bash
# Add uv to PATH
export PATH="$HOME/.local/bin:$PATH"

# Add to .bashrc for persistence
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

#### Python version mismatch

**Problem**: Error about Python 3.10+ required.

**Solution**:
```bash
# Check Python version
python3 --version

# Install Python 3.10 if needed
sudo apt update
sudo apt install python3.10 python3.10-venv

# Use specific Python version with uv
uv venv --python python3.10
```

### ROS2 Communication Issues

#### No topics visible

**Problem**: `ros2 topic list` shows no topics or very few topics.

**Solution**:
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Ensure it matches across all terminals
export ROS_DOMAIN_ID=42  # Or your chosen ID

# Check DDS discovery
ros2 multicast receive

# Try different DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# or
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

#### Service calls timeout

**Problem**: MCP tools timeout when calling ROS2 services.

**Solution**:
```bash
# Check if services are available
ros2 service list | grep autoware

# Test service directly
ros2 service call /api/operation_mode/get_state autoware_adapi_v1_msgs/srv/GetOperationModeState

# Increase timeout in code
response = await service.call_async(request, timeout_sec=10.0)
```

### MCP Server Issues

#### Server won't start

**Problem**: MCP server fails to start or crashes immediately.

**Solution**:
```bash
# Check if port is in use (if using HTTP transport)
lsof -i :3000

# Check ROS2 environment
source /opt/ros/humble/setup.bash

# Run with debug logging
AUTOWARE_MCP_LOG_LEVEL=DEBUG uv run autoware-mcp

# Check for Python import errors
uv run python -c "from autoware_mcp import server"
```

#### Claude Desktop connection fails

**Problem**: Claude Desktop cannot connect to MCP server.

**Solution**:
1. Verify server is running:
```bash
ps aux | grep autoware-mcp
```

2. Check Claude Desktop config:
```json
{
  "mcpServers": {
    "autoware": {
      "command": "uv",
      "args": ["run", "autoware-mcp"],
      "cwd": "/home/user/autoware-mcp",
      "env": {
        "ROS_DOMAIN_ID": "42"
      }
    }
  }
}
```

3. Test manual connection:
```bash
cd ~/autoware-mcp
uv run autoware-mcp
# Should see FastMCP banner
```

### Vehicle Control Issues

#### Vehicle not moving

**Problem**: Vehicle stays stationary despite autonomous mode.

**Diagnosis**:
```python
# Check operation mode
mode = await client.monitor_operation_mode()
print(f"Mode: {mode}")  # Should be "autonomous"

# Check route state
route = await client.get_current_route()
print(f"Route state: {route['state']}")  # Should be 2 (SET)

# Check motion state
motion = await client.monitor_motion_state()
print(f"Motion ready: {motion['ready']}")
```

**Solutions**:

1. **Localization not initialized**:
```python
await client.initialize_localization(initial_pose)
await asyncio.sleep(5)  # Wait for convergence
```

2. **Route not set properly**:
```python
# Clear and reset route
await client.call_ros2_service(
    "/api/routing/clear_route",
    "std_srvs/srv/Trigger",
    {}
)
await client.set_route(goal_pose)
```

3. **Motion not accepted**:
```python
await client.call_ros2_service(
    "/api/motion/accept_start",
    "std_srvs/srv/Trigger",
    {}
)
```

#### Vehicle oscillating or unstable

**Problem**: Vehicle moves erratically or oscillates.

**Solution**:
```bash
# Check control parameters
ros2 param list /control/trajectory_follower

# Reduce speed limits
ros2 param set /planning/scenario_planning/lane_driving max_velocity 2.0

# Check CPU load
top -p $(pgrep -f autoware)
```

### Launch Management Issues

#### Launch sessions not stopping

**Problem**: Launch sessions remain active after stop command.

**Solution**:
```python
# Force stop with SIGKILL
await client.stop_launch(session_id, force=True)

# Clean up orphaned sessions
await client.cleanup_orphans()

# Manual cleanup
killall -9 ros2
```

#### Generated launch files fail

**Problem**: AI-generated launch files don't work.

**Solution**:
```python
# Validate before running
result = await client.validate_launch_file(file_path)
if not result["success"]:
    print(f"Validation error: {result['error']}")

# Test with dry run
result = await client.test_launch_file(file_path, dry_run=True)

# Check syntax manually
python3 -m py_compile launch_file.launch.py
```

### Performance Issues

#### High CPU usage

**Problem**: System running slowly, high CPU usage.

**Solutions**:
```bash
# Reduce perception frequency
ros2 param set /perception/object_recognition update_rate 5.0

# Disable unnecessary visualizations
ros2 topic pub /control/debug/markers visualization_msgs/msg/MarkerArray "{}" --once

# Use release build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Memory leaks

**Problem**: Memory usage grows over time.

**Solution**:
```bash
# Monitor memory
watch -n 1 'ps aux | grep autoware | awk "{sum+=\$6} END {print sum/1024 \" MB\"}"'

# Restart periodically
systemctl restart autoware-mcp.service

# Profile Python memory
uv run python -m memory_profiler your_script.py
```

### Simulation Issues

#### Simulation won't start

**Problem**: Planning simulation fails to start.

**Solution**:
```bash
# Check if already running
./scripts/run_planning_simulation.sh status

# Force stop and restart
./scripts/run_planning_simulation.sh stop
sleep 5
./scripts/run_planning_simulation.sh start

# Check logs
tail -f /tmp/planning_simulation.log

# Verify map data
ls ~/autoware_map/sample-map-planning/
```

#### RViz not displaying

**Problem**: RViz window doesn't appear or is black.

**Solution**:
```bash
# Check display
echo $DISPLAY

# For SSH connections
export DISPLAY=:0

# For headless systems
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &

# GPU issues
export LIBGL_ALWAYS_SOFTWARE=1
```

## Diagnostic Commands

### System Health Check

```bash
# Complete health check script
cat << 'EOF' > health_check.sh
#!/bin/bash
echo "=== ROS2 Status ==="
ros2 doctor

echo -e "\n=== Active Nodes ==="
ros2 node list | wc -l

echo -e "\n=== Topic Activity ==="
ros2 topic list | wc -l

echo -e "\n=== Service Availability ==="
ros2 service list | grep -c autoware

echo -e "\n=== CPU Usage ==="
top -bn1 | grep autoware | head -5

echo -e "\n=== Memory Usage ==="
free -h

echo -e "\n=== Disk Usage ==="
df -h /

echo -e "\n=== Network ==="
ip addr show | grep inet
EOF

chmod +x health_check.sh
./health_check.sh
```

### Debug Logging

Enable verbose logging:

```python
# In Python code
import logging
logging.basicConfig(level=logging.DEBUG)

# Environment variable
export AUTOWARE_MCP_LOG_LEVEL=DEBUG

# ROS2 logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1
```

### Performance Profiling

```bash
# CPU profiling
py-spy record -o profile.svg -- uv run python your_script.py

# Memory profiling
mprof run uv run python your_script.py
mprof plot

# ROS2 profiling
ros2 run rqt_top rqt_top
```

## Error Messages

### Common Error Codes

| Error | Meaning | Solution |
|-------|---------|----------|
| `-32602` | Invalid params | Check parameter format |
| `-32603` | Internal error | Check server logs |
| `ECONNREFUSED` | Connection refused | Start MCP server |
| `ETIMEDOUT` | Operation timeout | Increase timeout |
| `ENOENT` | File not found | Check file paths |

### ROS2 Service Errors

| Error | Meaning | Solution |
|-------|---------|----------|
| `service not available` | Service doesn't exist | Check service name |
| `timeout exceeded` | Service call timeout | Check if node is running |
| `request type mismatch` | Wrong message type | Verify service type |

## Getting Help

If you're still experiencing issues:

1. **Check logs**:
```bash
# MCP server logs
journalctl -u autoware-mcp -f

# ROS2 logs
ros2 topic echo /rosout

# Autoware logs
tail -f ~/.ros/log/latest/*.log
```

2. **Enable debug mode**:
```bash
export AUTOWARE_MCP_DEBUG=1
export ROS_LOG_DIR=/tmp/ros_logs
```

3. **Collect diagnostic info**:
```bash
ros2 doctor --report > diagnostic_report.txt
```

4. **Report issue**:
- Include diagnostic report
- Provide minimal reproduction steps
- Share relevant log excerpts
- Specify versions (ROS2, Autoware, MCP)

## Prevention Tips

1. **Regular maintenance**:
   - Update dependencies: `uv sync`
   - Clean build: `colcon build --clean-first`
   - Clear logs: `rm -rf ~/.ros/log/*`

2. **Monitor resources**:
   - Set up alerts for high CPU/memory
   - Monitor disk space
   - Track network latency

3. **Test thoroughly**:
   - Run integration tests regularly
   - Test in simulation before real hardware
   - Have rollback procedures

4. **Document issues**:
   - Keep a log of issues and solutions
   - Share knowledge with team
   - Contribute fixes upstream