# Autoware MCP Integration

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Autoware](https://img.shields.io/badge/Autoware-0.45.1-orange.svg)](https://autoware.org/)

## Overview

The Autoware MCP Integration project provides a bridge between AI systems and the Autoware autonomous driving stack through the Model Context Protocol (MCP). This enables AI-driven mission planning, real-time vehicle control, and adaptive decision-making for autonomous vehicles.

### Key Features

- 🤖 **AI-Driven Planning**: Leverage AI capabilities for complex mission planning and decision-making
- 🚗 **Full Vehicle Control**: Complete control over Autoware's autonomous driving features
- 📊 **Real-Time Monitoring**: Concurrent monitoring of perception, planning, and vehicle state
- 🛡️ **Multi-Layer Safety**: Comprehensive safety validation at every level
- 🔄 **Adaptive Behavior**: Dynamic adjustment based on real-time conditions
- 🎯 **Mission Execution**: Support for complex multi-step missions with waypoints

## Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- Autoware 0.45.1
- Docker (optional)

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/autoware-mcp.git
cd autoware-mcp

# Install dependencies
pip install -r requirements.txt

# Setup Autoware (if not already installed)
# See: https://autowarefoundation.github.io/autoware-documentation/main/installation/
```

### Basic Usage

1. **Start Autoware**:
```bash
cd ~/autoware
source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/path/to/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

2. **Start MCP Server**:
```bash
cd autoware-mcp
python -m autoware_mcp.server --config config/default.yaml
```

3. **Connect AI Client**:
```python
from mcp import Client

# Connect to MCP server
client = Client("ws://localhost:8080")

# Plan and execute a route
await client.call_tool("plan_route", {
    "destination": {"x": 100.0, "y": 50.0, "z": 0.0},
    "constraints": {"avoid_highways": False}
})

# Set autonomous mode
await client.call_tool("set_operation_mode", {"mode": "autonomous"})
```

## Architecture

The system consists of three main layers:

```
AI Layer (Claude, GPT-4, etc.)
    ↓ MCP Protocol
MCP Server (Bridge)
    ↓ ROS2/AD API
Autoware Stack (Autonomous Driving)
```

For detailed architecture documentation, see [docs/01-architecture.md](docs/01-architecture.md).

## Available MCP Tools

### Mission Planning
- `plan_route` - Plan routes with waypoints and constraints
- `execute_mission` - Execute complex multi-step missions

### Vehicle Control
- `set_operation_mode` - Change between stop/autonomous/manual modes
- `control_vehicle` - Direct velocity and steering control
- `emergency_stop` - Immediate vehicle stop

### Monitoring
- `monitor_perception` - Real-time object detection stream
- `get_vehicle_state` - Current vehicle position and status
- `analyze_scene` - Scene understanding and analysis

### Planning
- `evaluate_trajectory` - Safety evaluation of planned path
- `request_replan` - Dynamic trajectory adjustment

For complete API documentation, see [docs/03-api-specification.md](docs/03-api-specification.md).

## Example Applications

### Urban Navigation
```python
# Plan a route through the city with multiple stops
mission = await client.call_tool("execute_mission", {
    "mission_id": "delivery_route",
    "steps": [
        {"action": "navigate_to", "parameters": {"location": "pickup_point"}},
        {"action": "wait", "parameters": {"duration": 30}},
        {"action": "navigate_to", "parameters": {"location": "delivery_point"}}
    ],
    "monitoring": ["pedestrian_detection", "traffic_monitoring"]
})
```

### Highway Driving
```python
# Monitor and adapt to highway traffic
monitor = await client.call_tool("monitor_perception", {
    "stream_id": "highway_monitor",
    "filters": {"object_types": ["vehicle"], "distance_threshold": 100},
    "frequency_hz": 20
})

# React to traffic conditions
async for update in monitor:
    if update.congestion_level > 0.7:
        await client.call_tool("request_replan", {"reason": "traffic"})
```

## Documentation

- [Architecture Overview](docs/01-architecture.md) - System design and framework choices
- [Component Design](docs/02-components.md) - Detailed component specifications
- [API Specification](docs/03-api-specification.md) - Complete API and message formats
- [Monitoring & Safety](docs/04-monitoring-safety.md) - Safety systems and monitoring
- [Development Guide](docs/05-development-guide.md) - Building, testing, and deployment

## Configuration

Create a configuration file `config/custom.yaml`:

```yaml
mcp_server:
  host: 0.0.0.0
  port: 8080
  
autoware:
  ros_domain_id: 42
  ad_api_url: http://localhost:8888
  
safety:
  max_velocity: 50.0  # m/s
  max_acceleration: 3.0  # m/s²
  emergency_deceleration: -8.0  # m/s²
```

## Development

### Running Tests

```bash
# Unit tests
pytest tests/unit -v

# Integration tests (requires Autoware)
pytest tests/integration -v

# Simulation tests
pytest tests/simulation -v

# All tests with coverage
pytest --cov=autoware_mcp --cov-report=html
```

### Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

## Safety Notice

⚠️ **WARNING**: This system is designed for research and development purposes. Always ensure:
- A safety driver is present during real vehicle testing
- Emergency stop mechanisms are properly configured
- All safety validations are enabled
- Testing is conducted in controlled environments

## Troubleshooting

### Common Issues

1. **ROS2 Connection Issues**:
```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

2. **MCP Server Not Responding**:
- Check firewall settings
- Verify Autoware is running
- Check logs: `tail -f logs/mcp_server.log`

3. **Performance Issues**:
- Monitor resource usage: `htop`, `nvidia-smi`
- Check ROS2 topics: `ros2 topic hz /topic_name`
- Review latency metrics in Grafana dashboard

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Autoware Foundation](https://www.autoware.org/) for the autonomous driving stack
- [Model Context Protocol](https://modelcontextprotocol.org/) contributors
- ROS2 and Open Robotics community

## Support

For issues and questions:
- GitHub Issues: [Create an issue](https://github.com/your-org/autoware-mcp/issues)
- Documentation: [Read the docs](https://your-docs-site.com)
- Community: [Join our Discord](https://discord.gg/your-invite)

## Roadmap

- [ ] Multi-vehicle coordination support
- [ ] Enhanced perception monitoring
- [ ] Cloud-based mission management
- [ ] V2X communication integration
- [ ] Advanced ML-based planning
- [ ] Simulation environment integration

---

**Note**: This project is under active development. APIs and features may change.