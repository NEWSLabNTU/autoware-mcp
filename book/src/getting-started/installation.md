# Installation

This guide will walk you through installing the Autoware MCP Server and its dependencies.

## System Requirements

### Hardware
- **CPU**: 8+ cores recommended (4 cores minimum)
- **RAM**: 16GB+ recommended (8GB minimum)
- **Storage**: 50GB+ free space
- **GPU**: NVIDIA GPU with CUDA support (optional, for perception)

### Software
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Docker**: 20.10+ (optional, for containerized deployment)

## Installation Methods

### Method 1: Quick Install (Recommended)

```bash
# Clone the repository
git clone https://github.com/your-org/autoware-mcp.git
cd autoware-mcp

# Install uv (Python package manager)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install dependencies
uv sync --all-extras --dev

# Verify installation
uv run autoware-mcp --help
```

### Method 2: Docker Installation

```bash
# Pull the Docker image
docker pull autoware-mcp:latest

# Run the container
docker run -it \
  --network host \
  --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  autoware-mcp:latest
```

### Method 3: Manual Installation

#### Step 1: Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

#### Step 2: Install Autoware (Optional)

If you want to run with the full Autoware stack:

```bash
# Follow the official Autoware installation guide
# https://autowarefoundation.github.io/autoware-documentation/main/installation/
```

#### Step 3: Install Python Dependencies

```bash
# Install uv package manager
curl -LsSf https://astral.sh/uv/install.sh | sh

# Clone and install Autoware MCP
git clone https://github.com/your-org/autoware-mcp.git
cd autoware-mcp
uv sync --all-extras --dev
```

## Environment Setup

### ROS2 Environment

Add to your `~/.bashrc`:

```bash
# ROS2 setup
source /opt/ros/humble/setup.bash

# Autoware setup (if installed)
source ~/autoware/install/setup.bash

# ROS Domain ID (change as needed)
export ROS_DOMAIN_ID=42

# DDS configuration (optional)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Python Environment

The project uses `uv` for dependency management:

```bash
# Activate the virtual environment
source .venv/bin/activate

# Or use uv run for commands
uv run autoware-mcp
```

## Verification

### Check ROS2 Installation

```bash
# Should show ROS2 topics
ros2 topic list

# Should show "humble"
echo $ROS_DISTRO
```

### Check MCP Server

```bash
# Test the MCP server
uv run autoware-mcp --help

# Run tests
uv run pytest tests/
```

### Check Autoware Connection (if installed)

```bash
# Start a test simulation
./scripts/run_planning_simulation.sh start

# In another terminal, check topics
ros2 topic list | grep autoware
```

## Configuration

### Basic Configuration

Create a configuration file at `~/.autoware-mcp/config.yaml`:

```yaml
# MCP Server Configuration
server:
  host: "localhost"
  port: 3000
  log_level: "INFO"

# ROS2 Configuration
ros2:
  domain_id: 42
  rmw_implementation: "rmw_cyclonedds_cpp"
  
# Autoware Configuration
autoware:
  workspace: "~/autoware"
  map_path: "~/autoware_map/sample-map-planning"
  vehicle_model: "sample_vehicle"
  sensor_model: "sample_sensor_kit"
```

### Environment Variables

```bash
# Required
export ROS_DOMAIN_ID=42

# Optional
export AUTOWARE_MCP_CONFIG="~/.autoware-mcp/config.yaml"
export AUTOWARE_MCP_LOG_LEVEL="DEBUG"
```

## Troubleshooting Installation

### Common Issues

#### 1. ROS2 not found
```bash
# Error: "ros2: command not found"
# Solution: Source ROS2 setup
source /opt/ros/humble/setup.bash
```

#### 2. Python version mismatch
```bash
# Error: "Python 3.10+ required"
# Solution: Install Python 3.10
sudo apt install python3.10 python3.10-venv
```

#### 3. uv command not found
```bash
# Error: "uv: command not found"
# Solution: Add uv to PATH
export PATH="$HOME/.local/bin:$PATH"
```

#### 4. ROS2 communication issues
```bash
# Check DDS discovery
ros2 multicast receive

# If issues, try different DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Next Steps

Once installation is complete:

1. [Configure Claude Desktop](./claude-desktop-setup.md) for AI integration
2. Follow the [Quick Start Guide](./quickstart.md) to run your first mission
3. Explore [examples](../user-guide/autonomous-driving.md) for practical usage

## Updating

To update to the latest version:

```bash
cd autoware-mcp
git pull
uv sync --all-extras --dev
```

## Uninstalling

To remove the installation:

```bash
# Remove the project directory
rm -rf autoware-mcp

# Remove uv (optional)
rm ~/.local/bin/uv

# Remove configuration
rm -rf ~/.autoware-mcp
```