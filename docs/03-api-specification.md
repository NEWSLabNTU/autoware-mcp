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

## 3. Autoware AD API Mappings

### 3.1 Service Mappings

| MCP Tool | AD API Service | ROS2 Fallback |
|----------|---------------|---------------|
| set_operation_mode | /api/operation_mode/change_to_* | /system/operation_mode/change |
| plan_route | /api/routing/set_route_points | /planning/mission_planning/route |
| control_vehicle | /api/control/command/* | /control/command/control_cmd |
| get_vehicle_state | /api/vehicle/kinematics | /localization/kinematic_state |

### 3.2 Topic Mappings

| Data Stream | AD API Topic | ROS2 Topic |
|-------------|-------------|------------|
| Perception Objects | /api/perception/objects | /perception/object_recognition/objects |
| Vehicle Status | /api/vehicle/status | /vehicle/status |
| Trajectory | - | /planning/scenario_planning/trajectory |
| Diagnostics | /api/system/diagnostics/status | /diagnostics_agg |

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