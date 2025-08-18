# Architecture Overview

## 1. Introduction

The Autoware MCP Integration project provides a bridge between AI systems (such as Claude) and the Autoware autonomous driving stack through the Model Context Protocol (MCP). This architecture enables AI-driven mission planning, real-time vehicle control, and adaptive decision-making while maintaining safety and performance requirements.

## 2. Design Principles

### 2.1 Core Principles

- **Separation of Concerns**: Clear boundaries between AI planning, MCP translation, and Autoware execution
- **Safety First**: Multi-layer safety validation at every level
- **Modularity**: Pluggable components for different capabilities
- **Concurrency**: Support for parallel monitoring and control tasks
- **Fail-Safe Design**: Graceful degradation and emergency fallbacks
- **Real-time Capable**: Meet latency requirements for vehicle control

### 2.2 Technology Choices

- **MCP Protocol**: Industry-standard protocol for AI-system integration
- **ROS2 Humble**: Native Autoware communication framework
- **Python/TypeScript**: MCP server implementation options
- **Async/Await**: Concurrent operation management
- **gRPC/WebSocket**: High-performance communication channels

## 3. System Architecture

### 3.1 High-Level Architecture

```
┌────────────────────────────────────────────────────────────┐
│                        AI Layer                            │
│  (Claude, GPT-4, or other AI with planning capabilities)   │
└────────────────────────────────────────────────────────────┘
                              │
                    MCP Protocol (JSON-RPC)
                              │
┌────────────────────────────────────────────────────────────┐
│                      MCP Server Layer                      │
│  ┌─────────────────────────────────────────────────────┐  │
│  │              MCP Protocol Handler                    │  │
│  └─────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────┐  │
│  │         Tool Registry & Execution Engine            │  │
│  └─────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────┐  │
│  │           Task & State Management                    │  │
│  └─────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
                              │
                    Internal API (Python/TS)
                              │
┌────────────────────────────────────────────────────────────┐
│                   Integration Layer                         │
│  ┌─────────────────────────────────────────────────────┐  │
│  │          Autoware AD API Client                      │  │
│  └─────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────┐  │
│  │           Direct ROS2 Interface                      │  │
│  └─────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────┐  │
│  │           Safety Validation Layer                    │  │
│  └─────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
                              │
                 ROS2 Topics/Services/Actions
                              │
┌────────────────────────────────────────────────────────────┐
│                    Autoware Stack                          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐    │
│  │Planning  │ │Control   │ │Perception│ │Localization│    │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘    │
└────────────────────────────────────────────────────────────┘
```

### 3.2 Component Interactions

#### 3.2.1 Command Flow (Top-Down)
1. AI generates high-level mission plan
2. MCP server decomposes into executable tasks
3. Tasks validated against safety constraints
4. Commands sent via AD API or ROS2
5. Autoware executes vehicle control

#### 3.2.2 Feedback Flow (Bottom-Up)
1. Autoware publishes sensor/state data
2. Integration layer aggregates information
3. MCP server processes into AI-consumable format
4. Streaming updates sent to AI
5. AI adapts plans based on feedback

## 4. Deployment Architecture

### 4.1 Single Machine Deployment

```yaml
# Docker Compose Configuration
services:
  mcp-server:
    container_name: autoware-mcp-server
    network: host  # Direct access to ROS2
    volumes:
      - /dev/shm:/dev/shm  # Shared memory for ROS2
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  
  autoware:
    container_name: autoware-universe
    network: host
    runtime: nvidia  # GPU support for perception
```

### 4.2 Distributed Deployment

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  AI Cloud   │────▶│ Edge Server │────▶│  Vehicle    │
│   Service   │◀────│ (MCP Server)│◀────│  (Autoware) │
└─────────────┘     └─────────────┘     └─────────────┘
     Internet          Low Latency          CAN/Ethernet
                         Network
```

## 5. Communication Patterns

### 5.1 Synchronous Operations
- Mission planning
- Route setting
- Mode changes
- Emergency stops

### 5.2 Asynchronous Streaming
- Perception data monitoring
- Vehicle state tracking
- Trajectory updates
- Diagnostic information

### 5.3 Event-Driven
- Obstacle detection alerts
- System failure notifications
- Goal reached events
- Safety interventions

## 6. Data Flow Architecture

### 6.1 Planning Data Flow

```
AI Planning Request
        │
        ▼
Mission Decomposition ──▶ Route Planning ──▶ Behavior Planning
        │                      │                    │
        ▼                      ▼                    ▼
    Validation            AD API Call         Trajectory Gen
        │                      │                    │
        ▼                      ▼                    ▼
    Execution            Route Active         Vehicle Motion
```

### 6.2 Monitoring Data Flow

```
Sensor Data ──▶ Perception ──▶ Object Tracking ──▶ Scene Analysis
     │              │               │                    │
     ▼              ▼               ▼                    ▼
Raw Topics    Detected Objs    Trajectories        AI Updates
     │              │               │                    │
     └──────────────┴───────────────┴────────────────────┘
                        MCP Aggregation
```

## 7. Scalability Considerations

### 7.1 Horizontal Scaling
- Multiple MCP servers for different capabilities
- Load balancing for AI requests
- Distributed monitoring systems

### 7.2 Vertical Scaling
- GPU acceleration for perception processing
- High-frequency control loops
- Memory optimization for large-scale maps

## 8. Security Architecture

### 8.1 Authentication
- MCP client authentication
- ROS2 DDS security plugins
- API key management

### 8.2 Authorization
- Role-based access control
- Command whitelist/blacklist
- Rate limiting

### 8.3 Data Protection
- Encrypted communication channels
- Secure storage of mission data
- Privacy-preserving telemetry

## 9. Fault Tolerance

### 9.1 Failure Modes
- AI connection loss → Local fallback mode
- MCP server crash → Emergency stop
- Autoware failure → Safety driver takeover
- Sensor failure → Degraded operation

### 9.2 Recovery Mechanisms
- Automatic reconnection
- State persistence and recovery
- Checkpoint-based mission resumption
- Graceful degradation

## 10. Performance Requirements

### 10.1 Latency Targets

| Operation         | Target | Maximum |
|-------------------|--------|---------|
| Control Command   | 20ms   | 50ms    |
| Perception Update | 50ms   | 100ms   |
| Planning Decision | 200ms  | 500ms   |
| Mission Planning  | 2s     | 5s      |

### 10.2 Throughput Requirements

| Data Stream      | Frequency | Bandwidth |
|------------------|-----------|-----------|
| Vehicle State    | 100 Hz    | 10 KB/s   |
| Object Tracking  | 30 Hz     | 100 KB/s  |
| Planning Updates | 10 Hz     | 50 KB/s   |
| Diagnostics      | 1 Hz      | 5 KB/s    |

## 11. Framework Integrations

### 11.1 AI Frameworks
- OpenAI API compatibility
- Anthropic Claude integration
- Custom LLM support
- Multi-agent systems

### 11.2 Simulation Frameworks
- AWSIM for urban testing
- CARLA for scenario generation
- Gazebo for sensor simulation
- Custom simulators via ROS2

### 11.3 Cloud Platforms
- AWS RoboMaker
- Azure IoT Edge
- Google Cloud Robotics
- Edge deployment solutions
