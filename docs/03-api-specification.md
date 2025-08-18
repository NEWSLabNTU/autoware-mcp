# API and Message Specification

## 1. MCP Tools API

### 1.1 Mission Planning Tools

#### plan_route
```typescript
{
  "name": "plan_route",
  "description": "Plan a route with optional waypoints and constraints",
  "inputSchema": {
    "type": "object",
    "properties": {
      "destination": {
        "type": "object",
        "properties": {
          "position": { "$ref": "#/definitions/Position" },
          "orientation": { "$ref": "#/definitions/Quaternion" }
        },
        "required": ["position"]
      },
      "waypoints": {
        "type": "array",
        "items": { "$ref": "#/definitions/Pose" }
      },
      "constraints": {
        "type": "object",
        "properties": {
          "avoid_highways": { "type": "boolean" },
          "avoid_tolls": { "type": "boolean" },
          "prefer_scenic": { "type": "boolean" },
          "max_duration_minutes": { "type": "number" },
          "departure_time": { "type": "string", "format": "date-time" }
        }
      }
    },
    "required": ["destination"]
  }
}
```

#### execute_mission
```typescript
{
  "name": "execute_mission",
  "description": "Execute a complex multi-step mission",
  "inputSchema": {
    "type": "object",
    "properties": {
      "mission_id": { "type": "string" },
      "steps": {
        "type": "array",
        "items": {
          "type": "object",
          "properties": {
            "action": { "type": "string" },
            "parameters": { "type": "object" },
            "conditions": { "type": "object" },
            "timeout_seconds": { "type": "number" }
          }
        }
      },
      "monitoring": {
        "type": "array",
        "items": { "type": "string" }
      },
      "safety_checks": { "type": "boolean", "default": true }
    },
    "required": ["mission_id", "steps"]
  }
}
```

### 1.2 Vehicle Control Tools

#### set_operation_mode
```typescript
{
  "name": "set_operation_mode",
  "description": "Change vehicle operation mode",
  "inputSchema": {
    "type": "object",
    "properties": {
      "mode": {
        "type": "string",
        "enum": ["stop", "autonomous", "local", "remote"]
      },
      "transition_time": {
        "type": "number",
        "description": "Maximum time to wait for transition (seconds)"
      }
    },
    "required": ["mode"]
  }
}
```

#### control_vehicle
```typescript
{
  "name": "control_vehicle",
  "description": "Send direct control commands to vehicle",
  "inputSchema": {
    "type": "object",
    "properties": {
      "control_type": {
        "type": "string",
        "enum": ["velocity", "acceleration", "steering", "combined"]
      },
      "velocity": {
        "type": "number",
        "minimum": -10,
        "maximum": 50,
        "description": "Target velocity in m/s"
      },
      "acceleration": {
        "type": "number",
        "minimum": -5,
        "maximum": 3,
        "description": "Target acceleration in m/s²"
      },
      "steering_angle": {
        "type": "number",
        "minimum": -0.7,
        "maximum": 0.7,
        "description": "Steering angle in radians"
      },
      "duration": {
        "type": "number",
        "description": "Duration to maintain command (seconds)"
      }
    },
    "required": ["control_type"]
  }
}
```

### 1.3 Monitoring Tools

#### monitor_perception
```typescript
{
  "name": "monitor_perception",
  "description": "Monitor perception data in real-time",
  "inputSchema": {
    "type": "object",
    "properties": {
      "stream_id": { "type": "string" },
      "filters": {
        "type": "object",
        "properties": {
          "object_types": {
            "type": "array",
            "items": {
              "type": "string",
              "enum": ["vehicle", "pedestrian", "cyclist", "unknown"]
            }
          },
          "distance_threshold": { "type": "number" },
          "velocity_threshold": { "type": "number" },
          "roi": { "$ref": "#/definitions/BoundingBox" }
        }
      },
      "frequency_hz": {
        "type": "number",
        "minimum": 1,
        "maximum": 30,
        "default": 10
      },
      "include_predictions": { "type": "boolean", "default": true }
    },
    "required": ["stream_id"]
  }
}
```

#### get_vehicle_state
```typescript
{
  "name": "get_vehicle_state",
  "description": "Get current vehicle state and metrics",
  "inputSchema": {
    "type": "object",
    "properties": {
      "include_diagnostics": { "type": "boolean", "default": false },
      "include_trajectory": { "type": "boolean", "default": false },
      "include_sensor_status": { "type": "boolean", "default": false }
    }
  }
}
```

### 1.4 Planning Tools

#### evaluate_trajectory
```typescript
{
  "name": "evaluate_trajectory",
  "description": "Evaluate safety and feasibility of current trajectory",
  "inputSchema": {
    "type": "object",
    "properties": {
      "horizon_seconds": {
        "type": "number",
        "minimum": 1,
        "maximum": 30,
        "default": 10
      },
      "checks": {
        "type": "array",
        "items": {
          "type": "string",
          "enum": ["collision", "dynamics", "rules", "comfort"]
        },
        "default": ["collision", "dynamics", "rules"]
      }
    }
  }
}
```

#### request_replan
```typescript
{
  "name": "request_replan",
  "description": "Request trajectory replanning",
  "inputSchema": {
    "type": "object",
    "properties": {
      "reason": {
        "type": "string",
        "enum": ["obstacle", "traffic", "route_change", "emergency"]
      },
      "constraints": {
        "type": "object",
        "properties": {
          "max_lateral_acceleration": { "type": "number" },
          "max_longitudinal_acceleration": { "type": "number" },
          "preferred_lane": { "type": "integer" }
        }
      },
      "priority": {
        "type": "string",
        "enum": ["normal", "high", "emergency"],
        "default": "normal"
      }
    },
    "required": ["reason"]
  }
}
```

## 2. Message Formats

### 2.1 Common Data Types

#### Position
```typescript
interface Position {
  x: number;        // meters
  y: number;        // meters
  z?: number;       // meters (optional)
  frame_id?: string; // coordinate frame
}
```

#### Pose
```typescript
interface Pose {
  position: Position;
  orientation: Quaternion;
}
```

#### Quaternion
```typescript
interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}
```

#### Timestamp
```typescript
interface Timestamp {
  seconds: number;
  nanoseconds: number;
}
```

### 2.2 Vehicle Messages

#### VehicleState
```typescript
interface VehicleState {
  timestamp: Timestamp;
  pose: Pose;
  velocity: {
    linear: Vector3;
    angular: Vector3;
  };
  acceleration: {
    linear: Vector3;
    angular: Vector3;
  };
  operation_mode: OperationMode;
  control_mode: ControlMode;
  fuel_level?: number;
  battery_level?: number;
}
```

#### VehicleCommand
```typescript
interface VehicleCommand {
  timestamp: Timestamp;
  control: {
    lateral: {
      steering_tire_angle: number;
      steering_tire_rotation_rate?: number;
    };
    longitudinal: {
      velocity?: number;
      acceleration?: number;
      jerk?: number;
    };
  };
  gear?: Gear;
  emergency_stop?: boolean;
}
```

### 2.3 Perception Messages

#### DetectedObject
```typescript
interface DetectedObject {
  id: string;
  timestamp: Timestamp;
  classification: {
    label: ObjectType;
    confidence: number;
  };
  kinematics: {
    pose: Pose;
    velocity: Vector3;
    acceleration?: Vector3;
  };
  shape: {
    type: "box" | "cylinder" | "polygon";
    dimensions: number[];
  };
  predicted_paths?: PredictedPath[];
}
```

#### PredictedPath
```typescript
interface PredictedPath {
  confidence: number;
  path: Array<{
    time_offset: number;  // seconds from now
    pose: Pose;
    velocity?: Vector3;
  }>;
}
```

### 2.4 Planning Messages

#### Route
```typescript
interface Route {
  id: string;
  waypoints: Waypoint[];
  total_distance: number;  // meters
  estimated_duration: number;  // seconds
  metadata?: {
    toll_cost?: number;
    includes_highway?: boolean;
    traffic_level?: "low" | "medium" | "high";
  };
}
```

#### Trajectory
```typescript
interface Trajectory {
  timestamp: Timestamp;
  points: TrajectoryPoint[];
  confidence: number;
  metadata?: {
    planner: string;
    constraints_satisfied: boolean;
    cost: number;
  };
}
```

#### TrajectoryPoint
```typescript
interface TrajectoryPoint {
  time_from_start: number;  // seconds
  pose: Pose;
  velocity: number;
  acceleration?: number;
  heading_rate?: number;
}
```

### 2.5 Event Messages

#### SystemEvent
```typescript
interface SystemEvent {
  id: string;
  timestamp: Timestamp;
  type: EventType;
  severity: "info" | "warning" | "error" | "critical";
  source: string;
  description: string;
  data?: any;
  required_action?: string;
}
```

#### SafetyEvent
```typescript
interface SafetyEvent extends SystemEvent {
  safety_action: "none" | "warning" | "slowdown" | "stop" | "emergency_stop";
  override_allowed: boolean;
  time_to_action?: number;  // seconds
}
```

## 3. Autoware AD API Integration

### 3.1 Operation Mode Management

| AD API Endpoint                              | Method  | MCP Tool                 | Description                    |
|----------------------------------------------|---------|--------------------------|--------------------------------|
| /api/operation_mode/change_to_autonomous     | Service | set_operation_mode       | Switch to autonomous driving   |
| /api/operation_mode/change_to_local          | Service | set_operation_mode       | Switch to local manual control |
| /api/operation_mode/change_to_remote         | Service | set_operation_mode       | Switch to remote control       |
| /api/operation_mode/change_to_stop           | Service | set_operation_mode       | Stop vehicle operation         |
| /api/operation_mode/enable_autoware_control  | Service | enable_autoware_control  | Enable Autoware control        |
| /api/operation_mode/disable_autoware_control | Service | disable_autoware_control | Disable Autoware control       |
| /api/operation_mode/state                    | Topic   | monitor_operation_mode   | Monitor operation mode state   |

### 3.2 Routing and Navigation

| AD API Endpoint                  | Method  | MCP Tool              | Description               |
|----------------------------------|---------|-----------------------|---------------------------|
| /api/routing/set_route           | Service | set_route             | Set route using goal pose |
| /api/routing/set_route_points    | Service | set_route_points      | Set route with waypoints  |
| /api/routing/change_route        | Service | change_route          | Modify current route      |
| /api/routing/change_route_points | Service | change_route_points   | Modify route waypoints    |
| /api/routing/clear_route         | Service | clear_route           | Clear current route       |
| /api/routing/route               | Topic   | get_current_route     | Get current route state   |
| /api/routing/state               | Topic   | monitor_routing_state | Monitor routing state     |

### 3.3 Localization

| AD API Endpoint                        | Method  | MCP Tool                   | Description                  |
|----------------------------------------|---------|----------------------------|------------------------------|
| /api/localization/initialize           | Service | initialize_localization    | Initialize pose estimation   |
| /api/localization/initialization_state | Topic   | monitor_localization_state | Monitor initialization state |

### 3.4 Vehicle Control

| AD API Endpoint                   | Method  | MCP Tool                  | Description               |
|-----------------------------------|---------|---------------------------|---------------------------|
| /api/control/command/velocity     | Stream  | send_velocity_command     | Send velocity control     |
| /api/control/command/acceleration | Stream  | send_acceleration_command | Send acceleration control |
| /api/control/command/steering     | Stream  | send_steering_command     | Send steering control     |
| /api/control/command/pedals       | Stream  | send_pedals_command       | Send pedal control        |
| /api/motion/accept_start          | Service | accept_start_request      | Accept motion start       |
| /api/motion/state                 | Topic   | monitor_motion_state      | Monitor motion state      |

### 3.5 Manual Control (Local/Remote)

| AD API Endpoint                        | Method  | MCP Tool                   | Description                |
|----------------------------------------|---------|----------------------------|----------------------------|
| /api/manual/local/command/*            | Stream  | send_local_manual_command  | Local manual control       |
| /api/manual/local/control_mode/select  | Service | select_local_control_mode  | Select local control mode  |
| /api/manual/local/control_mode/list    | Service | list_local_control_modes   | List available modes       |
| /api/manual/remote/command/*           | Stream  | send_remote_manual_command | Remote manual control      |
| /api/manual/remote/control_mode/select | Service | select_remote_control_mode | Select remote control mode |
| /api/manual/remote/operator/heartbeat  | Stream  | send_operator_heartbeat    | Operator heartbeat signal  |

### 3.6 Vehicle Information

| AD API Endpoint            | Method  | MCP Tool                   | Description                 |
|----------------------------|---------|----------------------------|-----------------------------|
| /api/vehicle/dimensions    | Service | get_vehicle_dimensions     | Get vehicle dimensions      |
| /api/vehicle/kinematics    | Stream  | monitor_vehicle_kinematics | Monitor kinematics          |
| /api/vehicle/status        | Stream  | monitor_vehicle_status     | Monitor vehicle status      |
| /api/vehicle/metrics       | Stream  | monitor_vehicle_metrics    | Monitor performance metrics |
| /api/vehicle/specs         | Service | get_vehicle_specs          | Get vehicle specifications  |
| /api/vehicle/doors/layout  | Service | get_door_layout            | Get door configuration      |
| /api/vehicle/doors/command | Service | control_doors              | Control vehicle doors       |
| /api/vehicle/doors/status  | Topic   | monitor_door_status        | Monitor door status         |

### 3.7 Perception

| AD API Endpoint         | Method | MCP Tool                   | Description                |
|-------------------------|--------|----------------------------|----------------------------|
| /api/perception/objects | Stream | monitor_perception_objects | Real-time object detection |

### 3.8 Planning

| AD API Endpoint                        | Method  | MCP Tool                  | Description               |
|----------------------------------------|---------|---------------------------|---------------------------|
| /api/planning/steering_factors         | Stream  | monitor_steering_factors  | Steering decision factors |
| /api/planning/velocity_factors         | Stream  | monitor_velocity_factors  | Velocity decision factors |
| /api/planning/cooperation/get_policies | Service | get_cooperation_policies  | Get cooperation policies  |
| /api/planning/cooperation/set_policies | Service | set_cooperation_policies  | Set cooperation policies  |
| /api/planning/cooperation/set_commands | Service | send_cooperation_commands | Send cooperation commands |

### 3.9 Fail-Safe System

| AD API Endpoint                     | Method  | MCP Tool             | Description            |
|-------------------------------------|---------|----------------------|------------------------|
| /api/fail_safe/mrm_state            | Topic   | monitor_mrm_state    | Monitor MRM state      |
| /api/fail_safe/list_mrm_description | Service | list_mrm_behaviors   | List MRM behaviors     |
| /api/fail_safe/mrm_request/send     | Service | request_mrm          | Request MRM activation |
| /api/fail_safe/mrm_request/list     | Topic   | monitor_mrm_requests | Monitor MRM requests   |
| /api/fail_safe/rti_state            | Topic   | monitor_rti_state    | Monitor RTI state      |

### 3.10 System Monitoring

| AD API Endpoint                | Method  | MCP Tool                  | Description               |
|--------------------------------|---------|---------------------------|---------------------------|
| /api/system/diagnostics/status | Stream  | monitor_diagnostics       | Real-time diagnostics     |
| /api/system/diagnostics/struct | Topic   | get_diagnostics_structure | Get diagnostics structure |
| /api/system/diagnostics/reset  | Service | reset_diagnostics         | Reset diagnostics         |
| /api/system/heartbeat          | Stream  | monitor_system_heartbeat  | System heartbeat          |
| /api/interface/version         | Service | get_api_version           | Get API version           |

### 3.11 Topic Fallback Mappings

| Data Category       | AD API Topic                   | ROS2 Fallback Topic                    |
|---------------------|--------------------------------|----------------------------------------|
| Perception Objects  | /api/perception/objects        | /perception/object_recognition/objects |
| Vehicle Status      | /api/vehicle/status            | /vehicle/status                        |
| Vehicle Kinematics  | /api/vehicle/kinematics        | /localization/kinematic_state          |
| Planning Trajectory | -                              | /planning/scenario_planning/trajectory |
| Diagnostics         | /api/system/diagnostics/status | /diagnostics                           |
| Operation Mode      | /api/operation_mode/state      | /system/operation_mode/state           |
| MRM State           | /api/fail_safe/mrm_state       | /system/fail_safe/mrm_state            |
| Routing State       | /api/routing/state             | /planning/mission_planning/state       |
| Steering Factors    | /api/planning/steering_factors | /planning/steering_factor/*            |
| Velocity Factors    | /api/planning/velocity_factors | /planning/velocity_factors/*           |

## 4. Streaming Protocols

### 4.1 WebSocket Streams

```typescript
interface StreamMessage {
  stream_id: string;
  sequence: number;
  timestamp: Timestamp;
  data: any;
  metadata?: {
    dropped_messages?: number;
    latency_ms?: number;
  };
}
```

### 4.2 Server-Sent Events (SSE)

```typescript
// SSE Format
interface SSEMessage {
  event: string;
  data: string;  // JSON stringified
  id?: string;
  retry?: number;
}
```

## 5. Error Responses

### 5.1 MCP Error Format

```typescript
interface MCPError {
  code: number;
  message: string;
  details?: {
    tool?: string;
    parameter?: string;
    constraint?: string;
    suggestion?: string;
  };
}
```

### 5.2 Error Codes

| Code | Name | Description |
|------|------|-------------|
| 1001 | TOOL_NOT_FOUND | Requested tool does not exist |
| 1002 | INVALID_PARAMETERS | Tool parameters validation failed |
| 1003 | EXECUTION_FAILED | Tool execution encountered error |
| 2001 | AUTOWARE_DISCONNECTED | Cannot reach Autoware system |
| 2002 | OPERATION_NOT_ALLOWED | Current state doesn't allow operation |
| 2003 | SAFETY_VIOLATION | Operation would violate safety constraints |
| 3001 | STREAM_NOT_FOUND | Requested stream does not exist |
| 3002 | SUBSCRIPTION_FAILED | Cannot subscribe to data stream |

## 6. Configuration Messages

### 6.1 System Configuration

```typescript
interface SystemConfig {
  mcp_server: {
    host: string;
    port: number;
    ssl_enabled: boolean;
  };
  autoware: {
    ros_domain_id: number;
    ad_api_url: string;
    namespace: string;
  };
  safety: {
    max_velocity: number;
    max_acceleration: number;
    min_following_distance: number;
    emergency_deceleration: number;
  };
  monitoring: {
    perception_frequency_hz: number;
    planning_frequency_hz: number;
    diagnostics_frequency_hz: number;
  };
}
```

### 6.2 Tool Configuration

```typescript
interface ToolConfig {
  tool_name: string;
  enabled: boolean;
  parameters: {
    [key: string]: any;
  };
  rate_limit?: {
    calls_per_minute: number;
    burst_size: number;
  };
}
```
