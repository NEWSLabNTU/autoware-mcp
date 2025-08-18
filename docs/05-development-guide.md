# Development and Testing Guide

## 1. Development Environment Setup

### 1.1 Prerequisites

```bash
# System Requirements
- Ubuntu 22.04 LTS (recommended)
- Python 3.10+
- Node.js 18+ (for TypeScript implementation)
- Docker 24+
- NVIDIA GPU with CUDA 11.8+ (for Autoware perception)

# ROS2 Humble Installation
sudo apt update && sudo apt install -y \
  software-properties-common \
  curl \
  gnupg \
  lsb-release

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop-full
```

### 1.2 Project Setup

```bash
# Clone repository
git clone https://github.com/your-org/autoware-mcp.git
cd autoware-mcp

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt

# Install development dependencies
pip install -r requirements-dev.txt

# Setup pre-commit hooks
pre-commit install
```

### 1.3 Autoware Setup

```bash
# Clone Autoware
mkdir -p ~/autoware && cd ~/autoware
git clone https://github.com/autowarefoundation/autoware.git -b release/2024.03

# Install dependencies
cd autoware
./setup-dev-env.sh

# Build Autoware
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source environment
source install/setup.bash
```

## 2. Building the Project

### 2.1 Python Implementation

```bash
# Build Python package
cd autoware-mcp
python setup.py build

# Install in development mode
pip install -e .

# Run type checking
mypy src/

# Run linting
ruff check src/
black src/ --check
```

### 2.2 TypeScript Implementation

```bash
# Install dependencies
npm install

# Build TypeScript
npm run build

# Watch mode for development
npm run dev

# Type checking
npm run type-check
```

### 2.3 Docker Build

```dockerfile
# Dockerfile
FROM ros:humble-ros-base

# Install Python and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Copy project
WORKDIR /app
COPY . .

# Install Python dependencies
RUN pip3 install -r requirements.txt

# Build and install package
RUN python3 setup.py install

# Entry point
CMD ["python3", "-m", "autoware_mcp.server"]
```

```bash
# Build Docker image
docker build -t autoware-mcp:latest .

# Run with host network (for ROS2 communication)
docker run --rm -it \
  --network host \
  --ipc host \
  -v /dev/shm:/dev/shm \
  -e ROS_DOMAIN_ID=42 \
  autoware-mcp:latest
```

## 3. Testing Strategy

### 3.1 Unit Testing

```python
# tests/test_safety_validator.py
import pytest
from autoware_mcp.safety import SafetyValidator

class TestSafetyValidator:
    @pytest.fixture
    def validator(self):
        return SafetyValidator()
    
    def test_velocity_validation(self, validator):
        """Test velocity constraint validation"""
        # Valid velocity
        result = validator.validate_velocity(10.0)
        assert result.is_valid
        
        # Excessive velocity
        result = validator.validate_velocity(100.0)
        assert not result.is_valid
        assert "velocity" in result.error
    
    def test_acceleration_validation(self, validator):
        """Test acceleration constraint validation"""
        # Valid acceleration
        result = validator.validate_acceleration(2.0)
        assert result.is_valid
        
        # Excessive acceleration
        result = validator.validate_acceleration(10.0)
        assert not result.is_valid
```

### 3.2 Integration Testing

```python
# tests/integration/test_autoware_integration.py
import asyncio
import pytest
from autoware_mcp.client import AutowareClient

@pytest.mark.integration
class TestAutowareIntegration:
    @pytest.fixture
    async def client(self):
        client = AutowareClient()
        await client.connect()
        yield client
        await client.disconnect()
    
    @pytest.mark.asyncio
    async def test_set_route(self, client):
        """Test route setting through AD API"""
        # Set a simple route
        result = await client.set_route(
            goal={"x": 100.0, "y": 50.0, "z": 0.0}
        )
        assert result.success
        
        # Verify route is active
        state = await client.get_routing_state()
        assert state == "SET"
    
    @pytest.mark.asyncio
    async def test_operation_mode_change(self, client):
        """Test operation mode transitions"""
        # Start in STOP mode
        await client.set_operation_mode("stop")
        
        # Transition to AUTONOMOUS
        result = await client.set_operation_mode("autonomous")
        assert result.success
        
        # Verify mode change
        mode = await client.get_operation_mode()
        assert mode == "autonomous"
```

### 3.3 Simulation Testing

```python
# tests/simulation/test_scenarios.py
import pytest
from autoware_mcp.simulation import ScenarioRunner

class TestSimulationScenarios:
    @pytest.mark.simulation
    async def test_urban_navigation(self):
        """Test urban navigation scenario"""
        runner = ScenarioRunner("urban_navigation")
        
        # Load scenario
        await runner.load_scenario("scenarios/urban_crosswalk.yaml")
        
        # Execute scenario
        result = await runner.run()
        
        # Verify success criteria
        assert result.reached_goal
        assert result.collision_count == 0
        assert result.traffic_violations == 0
        assert result.completion_time < 300  # seconds
    
    @pytest.mark.simulation
    async def test_emergency_stop(self):
        """Test emergency stop scenario"""
        runner = ScenarioRunner("emergency_stop")
        
        # Setup scenario with obstacle
        await runner.setup_obstacle(distance=20.0, velocity=0.0)
        
        # Start driving
        await runner.start_autonomous_driving()
        
        # Wait for emergency stop
        await runner.wait_for_stop(timeout=5.0)
        
        # Verify safe stop
        distance_to_obstacle = await runner.get_distance_to_obstacle()
        assert distance_to_obstacle > 2.0  # meters
```

### 3.4 Performance Testing

```python
# tests/performance/test_latency.py
import time
import statistics
import pytest

class TestPerformance:
    @pytest.mark.performance
    async def test_command_latency(self, mcp_server):
        """Test MCP command latency"""
        latencies = []
        
        for _ in range(100):
            start = time.perf_counter()
            await mcp_server.call_tool("get_vehicle_state", {})
            latency = (time.perf_counter() - start) * 1000
            latencies.append(latency)
        
        # Calculate statistics
        avg_latency = statistics.mean(latencies)
        p95_latency = statistics.quantiles(latencies, n=20)[18]  # 95th percentile
        
        # Assert performance requirements
        assert avg_latency < 50  # ms
        assert p95_latency < 100  # ms
    
    @pytest.mark.performance
    async def test_streaming_throughput(self, mcp_server):
        """Test perception streaming throughput"""
        message_count = 0
        start_time = time.time()
        
        # Subscribe to perception stream
        async for msg in mcp_server.stream("perception_objects", duration=10.0):
            message_count += 1
        
        # Calculate throughput
        duration = time.time() - start_time
        throughput = message_count / duration
        
        # Assert minimum throughput
        assert throughput >= 10  # Hz
```

## 4. Development Workflow

### 4.1 Local Development

```bash
# Terminal 1: Run Autoware
cd ~/autoware/autoware
source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml map_path:=/path/to/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

# Terminal 2: Run MCP Server
cd ~/autoware-mcp
source venv/bin/activate
python -m autoware_mcp.server --config config/development.yaml

# Terminal 3: Run MCP Client (for testing)
python -m autoware_mcp.client.test_client
```

### 4.2 Debugging

```python
# Enable debug logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Use debugger
import pdb; pdb.set_trace()

# Or use IPython for interactive debugging
from IPython import embed; embed()

# Async debugging with aiomonitor
import aiomonitor
aiomonitor.start_monitor(loop=asyncio.get_event_loop())
```

### 4.3 Profiling

```python
# Profile CPU usage
import cProfile
import pstats

profiler = cProfile.Profile()
profiler.enable()
# ... code to profile ...
profiler.disable()

stats = pstats.Stats(profiler)
stats.sort_stats('cumulative')
stats.print_stats(20)

# Profile memory usage
from memory_profiler import profile

@profile
def memory_intensive_function():
    # ... code ...
    pass

# Async profiling
import yappi
yappi.set_clock_type("cpu")
yappi.start()
# ... async code ...
yappi.stop()
yappi.get_func_stats().print_all()
```

## 5. Continuous Integration

### 5.1 GitHub Actions Workflow

```yaml
# .github/workflows/ci.yml
name: CI

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-22.04
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install -r requirements-dev.txt
    
    - name: Run linting
      run: |
        ruff check src/
        black src/ --check
        mypy src/
    
    - name: Run unit tests
      run: |
        pytest tests/unit -v --cov=autoware_mcp
    
    - name: Upload coverage
      uses: codecov/codecov-action@v3
      with:
        file: ./coverage.xml

  integration:
    runs-on: ubuntu-22.04
    needs: test
    
    services:
      autoware:
        image: autoware/autoware:latest
        options: --privileged
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Run integration tests
      run: |
        docker-compose -f docker-compose.test.yml up --abort-on-container-exit
```

### 5.2 Pre-commit Hooks

```yaml
# .pre-commit-config.yaml
repos:
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.1.0
    hooks:
      - id: ruff
        args: [--fix]
      
  - repo: https://github.com/psf/black
    rev: 23.9.1
    hooks:
      - id: black
      
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v1.5.1
    hooks:
      - id: mypy
        additional_dependencies: [types-all]
```

## 6. Deployment

### 6.1 Production Build

```bash
# Build production Docker image
docker build -f Dockerfile.prod -t autoware-mcp:prod .

# Run with production config
docker run -d \
  --name autoware-mcp \
  --restart unless-stopped \
  --network host \
  -v /etc/autoware-mcp:/config \
  -e CONFIG_FILE=/config/production.yaml \
  autoware-mcp:prod
```

### 6.2 Monitoring Setup

```yaml
# docker-compose.monitoring.yml
version: '3.8'

services:
  prometheus:
    image: prom/prometheus
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
    ports:
      - "9090:9090"
  
  grafana:
    image: grafana/grafana
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    ports:
      - "3000:3000"
  
  loki:
    image: grafana/loki
    ports:
      - "3100:3100"
```

## 7. Troubleshooting

### 7.1 Common Issues

```bash
# ROS2 connection issues
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Permission issues
sudo usermod -aG docker $USER
sudo chmod 666 /var/run/docker.sock

# Memory issues
echo 2 | sudo tee /proc/sys/vm/overcommit_memory
sudo sysctl -w vm.max_map_count=262144
```

### 7.2 Debug Tools

```bash
# ROS2 debugging
ros2 topic list
ros2 topic echo /topic_name
ros2 service list
ros2 node list
ros2 param list

# Network debugging
netstat -tuln | grep 9090
tcpdump -i any port 9090

# Process debugging
htop
iotop
nvidia-smi
```

## 8. Best Practices

### 8.1 Code Style

```python
# Follow PEP 8 and use type hints
from typing import Optional, List, Dict
import asyncio

async def process_command(
    command: str,
    parameters: Optional[Dict[str, any]] = None
) -> Dict[str, any]:
    """Process MCP command.
    
    Args:
        command: The command to execute
        parameters: Optional command parameters
    
    Returns:
        Command execution result
    """
    # Implementation
    pass
```

### 8.2 Error Handling

```python
# Use custom exceptions
class AutowareMCPError(Exception):
    """Base exception for Autoware MCP"""
    pass

class SafetyViolationError(AutowareMCPError):
    """Raised when safety constraints are violated"""
    pass

# Proper error handling
try:
    result = await execute_command(cmd)
except SafetyViolationError as e:
    logger.error(f"Safety violation: {e}")
    await emergency_stop()
except Exception as e:
    logger.exception("Unexpected error")
    raise
```

### 8.3 Testing Guidelines

1. Write tests before fixing bugs
2. Maintain > 80% code coverage
3. Use meaningful test names
4. Mock external dependencies
5. Test edge cases and error conditions
6. Run tests locally before pushing