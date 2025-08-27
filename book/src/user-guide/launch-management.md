# Launch Session Management Architecture Analysis for Autoware MCP

## Executive Summary

This report analyzes the requirements and implementation strategies for comprehensive launch session management in the Autoware MCP environment. The goal is to provide AI agents with robust control over ROS2 launch files while ensuring system stability and proper cleanup. This design emphasizes persistent process tracking through PID/PGID files and supports iterative development workflows where AI agents can generate, test, fix, and redeploy launch configurations.

## Current State Analysis

### ROS2 Launch System Overview

ROS2 uses a Python-based launch system that:
- Spawns multiple processes and nodes as defined in launch files
- Maintains parent-child process relationships
- Handles node lifecycle management
- Provides event-based architecture for monitoring

### Gap Analysis

Currently, the Autoware MCP server lacks:
- Persistent tracking of launch sessions across MCP reconnections
- Granular control over launch file execution states
- Orphan process prevention mechanisms
- Comprehensive session introspection capabilities

## Proposed Architecture

### 1. Launch Session Tracking

**Session Registry Design**

A persistent session registry should maintain:
- **Session ID**: Unique identifier for each launch instance
- **Launch File Path**: Source launch file location
- **Process Tree**: Complete hierarchy of spawned processes
- **State Machine**: Current execution state (running, paused, stopped)
- **Resource Metrics**: CPU, memory, and network usage per session
- **Creation Metadata**: Timestamp, initiator, parameters used

**Status Report Components**

Each session status report should include:
- Total number of active nodes
- Process health indicators (healthy/warning/critical)
- Inter-node communication statistics
- Resource consumption summary
- Uptime and last state transition time
- Dependencies and conflicts with other sessions

### 2. Launch Lifecycle Management

**State Transitions**

Launch sessions should support the following states:

- **INITIALIZED**: Launch file parsed, resources allocated
- **STARTING**: Nodes being spawned sequentially
- **RUNNING**: All nodes active and healthy
- **PAUSED**: Nodes suspended but state preserved
- **RESUMING**: Transitioning from paused to running
- **STOPPING**: Graceful shutdown in progress
- **TERMINATED**: All processes cleaned up
- **ERROR**: Abnormal termination or critical failure

**Start Operation**

Starting a launch file should:
1. Validate launch file syntax and dependencies
2. Check for resource conflicts with existing sessions
3. Create session entry with unique ID
4. Spawn processes with proper grouping for lifecycle management
5. Monitor startup completion and health checks
6. Transition to RUNNING state once all nodes are healthy

**Stop Operation**

Stopping should follow this sequence:
1. Send SIGTERM to all child processes
2. Wait for graceful shutdown (configurable timeout)
3. Force kill (SIGKILL) any remaining processes
4. Clean up shared memory and lock files
5. Update session registry to TERMINATED state
6. Archive logs for post-mortem analysis

**Pause/Resume Operations**

Pause functionality considerations:
- **Signal-based approach**: Send SIGSTOP to process group
- **State preservation**: Maintain message queues and buffers
- **Partial pause**: Ability to pause specific nodes while others continue
- **Resume synchronization**: Ensure coordinated restart to prevent message loss

### 3. MCP Server Lifecycle Integration

**PID/PGID File Management**

The MCP server maintains persistent process tracking through a structured file system in the current working directory, ensuring isolation between users and multiple server instances:

```
./.autoware-mcp/
├── instance_<port>_<pid>/     # Unique per MCP server instance
│   ├── sessions/
│   │   ├── session_<uuid>/
│   │   │   ├── session.pid       # Main launch process PID
│   │   │   ├── session.pgid      # Process group ID
│   │   │   ├── nodes.json        # Individual node PIDs and metadata
│   │   │   ├── launch.yaml       # Original launch configuration
│   │   │   └── state.json        # Current session state
│   │   └── ...
│   ├── mcp_server.pid            # This MCP server's PID
│   ├── mcp_server.port           # Port number for uniqueness
│   └── cleanup.lock              # Lock file for cleanup operations
├── generated/                    # AI-generated launch files and nodes
│   ├── launches/
│   │   ├── perception_v1.launch.py
│   │   ├── perception_v2.launch.py
│   │   └── ...
│   ├── nodes/
│   │   ├── custom_detector.py
│   │   └── ...
│   └── configs/
│       ├── params.yaml
│       └── ...
└── logs/                        # Session logs and diagnostics
    └── session_<uuid>/
        ├── stdout.log
        ├── stderr.log
        └── diagnostics.json
```

**Instance Isolation Strategy**

Each MCP server instance creates a unique subdirectory using:
- **Port number**: The MCP server's listening port (e.g., 5000, 5001)
- **Process ID**: The MCP server's PID for additional uniqueness
- **Example**: `instance_5000_12345/` for server on port 5000 with PID 12345

This ensures:
- Multiple users can run MCP servers in the same system without conflicts
- A single user can run multiple MCP servers for different AI agents
- Each project directory maintains its own MCP runtime data
- Generated code stays with the project for version control

**File Content Structure**

- **session.pid**: Single line containing the main launch process PID
- **session.pgid**: Process group ID for bulk signaling
- **nodes.json**: Detailed mapping:
  ```json
  {
    "nodes": [
      {
        "name": "perception_node",
        "pid": 12345,
        "started_at": "2025-01-20T10:30:00Z",
        "restart_count": 0,
        "critical": true
      }
    ],
    "last_updated": "2025-01-20T10:35:00Z"
  }
  ```

**Cleanup Mechanisms**

When MCP server closes:

1. **Graceful Cleanup Path**:
   - Read all session directories under `./.autoware-mcp/instance_<port>_<pid>/sessions/`
   - For each session, send SIGTERM to PGID from `session.pgid`
   - Wait for process termination (max 10 seconds)
   - Remove session directory after successful cleanup
   - Log cleanup status to `./.autoware-mcp/logs/cleanup_<timestamp>.log`

2. **Emergency Cleanup Path**:
   - Signal handler (SIGINT, SIGTERM, SIGHUP) triggers emergency cleanup
   - Read all PGID files from this instance's directory only
   - Send SIGKILL after grace period to processes owned by this instance
   - Write crash dump to `./.autoware-mcp/logs/crash_<timestamp>.log`
   - Leave PID files for post-mortem analysis

3. **Recovery on Restart**:
   - Scan `./.autoware-mcp/instance_*/` directories
   - Identify stale instances (MCP server PID no longer exists)
   - For each orphaned session in stale instances:
     - Check if processes still exist via `/proc/<pid>`
     - If alive and PGID matches, offer adoption or termination
     - If dead, clean up stale files
   - Clean up entire stale instance directory after handling
   - Log recovery actions with details

**Crash Recovery Procedure**

On MCP server startup:
```
1. Determine instance directory: ./.autoware-mcp/instance_<port>_<pid>/
2. Check for existing instances in ./.autoware-mcp/:
   - For each instance_<port>_<oldpid>/ directory:
     a. Read mcp_server.pid
     b. Check if process still running
     c. If running on same port, refuse to start
     d. If not running, mark as stale
3. For stale instances:
   - Read all session PGID files
   - Send SIGTERM to each PGID
   - Wait 5 seconds
   - Send SIGKILL to remaining processes
   - Move stale instance to ./.autoware-mcp/archived/<timestamp>/
4. Create new instance directory
5. Write mcp_server.pid and mcp_server.port
```

**Persistent State Management**

State persistence strategy:
- Store session metadata in `./.autoware-mcp/instance_<port>_<pid>/sessions.db` (SQLite)
- Keep database local to project directory for portability
- Sync PID files with database every 5 seconds
- Include process PIDs, PGIDs, and last known states
- Implement journaling for crash consistency
- Regular checkpointing during execution
- Archive completed sessions to `./.autoware-mcp/archived/` for history

### 4. Single Node vs. Launch File Execution

**Analysis of Single Node Execution**

Single node execution should be treated as a specialized case of launch execution for the following reasons:

1. **Consistency**: Unified interface reduces complexity
2. **Dependency Management**: Even single nodes may have hidden dependencies
3. **Lifecycle Management**: Same monitoring and control requirements
4. **Resource Tracking**: Identical need for resource monitoring

**Recommended Approach**

Implement single node execution as:
- Auto-generated minimal launch file
- Simplified parameter passing
- Quick-start templates for common nodes
- Same session management infrastructure

This provides consistency while offering convenience methods for simple cases.

## Implementation Considerations

### Process Group Management

**Linux Process Groups**
- Utilize `setpgid()` for process grouping
- Enable group-wide signal delivery
- Implement cgroup integration for resource limits
- Monitor using `/proc` filesystem

### Monitoring Infrastructure

**Health Check Strategy**
- Periodic liveness probes
- Node-specific health topics
- Deadline monitoring for critical nodes
- Automatic restart policies

**Resource Monitoring**
- Sample at configurable intervals (default 1Hz)
- Track trends for anomaly detection
- Alert on threshold violations
- Historical data for optimization

### Error Handling

**Failure Scenarios**

1. **Node Crash**: 
   - Detect via process monitoring
   - Attempt automatic restart (configurable)
   - Escalate to session failure if critical

2. **Resource Exhaustion**:
   - Preemptive monitoring and alerting
   - Graceful degradation strategies
   - Priority-based resource allocation

3. **Communication Failure**:
   - Dead peer detection
   - Message queue overflow handling
   - Network partition recovery

### Security Considerations

**Access Control**
- Session ownership model
- Permission levels for operations
- Audit logging for all actions
- Sandboxing for untrusted launches

**Resource Limits**
- Maximum sessions per user
- CPU/memory quotas
- Network bandwidth throttling
- Disk usage constraints

## Recommended MCP Tool Interface

The following tools should be exposed through MCP:

### Core Session Management
1. **list_launch_sessions**: Comprehensive session enumeration
2. **get_session_status**: Detailed single session report including PID/PGID info
3. **start_launch**: Initialize and run launch file, returns session UUID
4. **stop_launch**: Graceful session termination via PGID signaling
5. **pause_launch**: Suspend session execution (SIGSTOP to PGID)
6. **resume_launch**: Continue paused session (SIGCONT to PGID)
7. **restart_launch**: Stop and start in one operation
8. **get_session_logs**: Retrieve session output from buffered logs
9. **set_session_parameters**: Runtime parameter updates
10. **cleanup_orphans**: Remove abandoned processes using PID files

### Launch Generation and Development
11. **generate_launch_file**: Create launch files in `./.autoware-mcp/generated/launches/`
12. **validate_launch_file**: Syntax and dependency checking
13. **generate_node_config**: Create configs in `./.autoware-mcp/generated/configs/`
14. **generate_custom_node**: Create Python/C++ nodes in `./.autoware-mcp/generated/nodes/`
15. **test_launch_file**: Dry-run mode to validate without starting nodes
16. **diff_launch_sessions**: Compare two launch configurations
17. **get_launch_errors**: Retrieve diagnostics from `./.autoware-mcp/logs/`
18. **list_generated_files**: Show all AI-generated files in project
19. **commit_generated_files**: Stage generated files for git commit

## Workflow Examples

### Workflow 1: Iterative Development Cycle

This workflow demonstrates how an AI agent can iteratively develop and refine a perception pipeline:

**Step 1: Generate Initial Launch File**
```
AI Agent: "I need to create a perception pipeline for urban driving"
→ generate_launch_file(
    template="perception_pipeline",
    components=["lidar_processing", "camera_detection", "fusion"]
  )
→ Returns: launch_file_path = "./.autoware-mcp/generated/launches/perception_v1.launch.py"
→ File is created in project directory for version control
```

**Step 2: Start the Launch Session**
```
AI Agent: "Let me test this configuration"
→ start_launch(file_path="./.autoware-mcp/generated/launches/perception_v1.launch.py")
→ Returns: session_id = "abc123-def456"
→ Creates: ./.autoware-mcp/instance_5000_12345/sessions/abc123-def456/
           with session.pid=54321, session.pgid=54320
→ Logs output to: ./.autoware-mcp/logs/session_abc123-def456/
```

**Step 3: Monitor and Detect Issues**
```
AI Agent: "Checking for errors in the pipeline"
→ get_session_status(session_id="abc123-def456")
→ Returns: {"state": "ERROR", "failed_nodes": ["camera_detection"]}
→ get_launch_errors(session_id="abc123-def456")
→ Returns: "camera_detection: Missing parameter 'model_path'"
```

**Step 4: Fix and Regenerate**
```
AI Agent: "I need to add the model path parameter"
→ stop_launch(session_id="abc123-def456")
  (Sends SIGTERM to PGID 54320, waits, cleans up)
→ generate_node_config(
    node="camera_detection",
    params={"model_path": "./models/yolov5.pt"}  # Relative to project
  )
→ Saves to: ./.autoware-mcp/generated/configs/camera_detection.yaml
→ generate_launch_file(
    template="perception_pipeline",
    components=[...],
    configs={"camera_detection": node_config}
  )
→ Returns: launch_file_path = "./.autoware-mcp/generated/launches/perception_v2.launch.py"
→ Previous version kept for rollback if needed
```

**Step 5: Restart with Fixed Configuration**
```
→ start_launch(file_path="./.autoware-mcp/generated/launches/perception_v2.launch.py")
→ Returns: session_id = "xyz789-ghi012"
→ get_session_status(session_id="xyz789-ghi012")
→ Returns: {"state": "RUNNING", "nodes": 3, "healthy": true}
→ All generated files remain in project for git commit
```

### Workflow 2: Complex System Composition

**Step 1: Generate Multiple Component Launches**
```
AI Agent: "Building complete autonomous system"
→ generate_launch_file(name="perception", components=[...])
→ generate_launch_file(name="planning", components=[...])
→ generate_launch_file(name="control", components=[...])
→ generate_launch_file(
    name="main_system",
    includes=["perception", "planning", "control"],
    remappings={...}
  )
```

**Step 2: Progressive Bring-up**
```
→ start_launch("perception") → session_1
→ wait_for_healthy(session_1)
→ start_launch("planning") → session_2
→ wait_for_healthy(session_2)
→ start_launch("control") → session_3
```

**Step 3: Handle Cascading Failures**
```
If perception fails:
→ get_launch_errors(session_1)
→ pause_launch(session_2)  # Sends SIGSTOP to planning PGID
→ pause_launch(session_3)  # Sends SIGSTOP to control PGID
→ fix_and_restart(session_1)
→ resume_launch(session_2)  # Sends SIGCONT to planning PGID
→ resume_launch(session_3)  # Sends SIGCONT to control PGID
```

### Workflow 3: Crash Recovery

**Scenario: MCP Server Crashes**
```
1. MCP server (instance_5000_12345) terminates unexpectedly
2. PID files remain in ./.autoware-mcp/instance_5000_12345/sessions/
3. ROS nodes continue running with their PGIDs
4. Generated files remain safe in ./.autoware-mcp/generated/
```

**Recovery Process**
```
On MCP restart (new instance_5000_67890):
→ cleanup_orphans()
  - Scans ./.autoware-mcp/ for stale instances
  - Finds instance_5000_12345 with dead MCP server PID
  - Reads all session.pgid files from stale instance
  - For each PGID:
    - Check if processes exist
    - Prompt: "Found orphaned session abc123 (3 nodes) from previous instance. Action?"
    - Options: adopt, terminate, ignore
  - If terminate: Send SIGTERM to PGID, wait, SIGKILL if needed
  - Move stale instance to ./.autoware-mcp/archived/
  - Generated files remain untouched in ./.autoware-mcp/generated/
```

### Workflow 4: Performance Optimization Loop

**Iterative Tuning Process**
```
1. start_launch(baseline_config) → baseline_session
2. Monitor performance metrics
3. While performance < target:
   a. Analyze bottlenecks via get_session_status()
   b. Generate optimized config
   c. stop_launch(current_session)
   d. start_launch(optimized_config) → new_session
   e. Compare metrics
   f. Keep or rollback based on results
```

## Migration Path

### Phase 1: Basic Management
- Implement start/stop functionality
- Basic process tracking
- Simple cleanup on exit

### Phase 2: Advanced Control
- Add pause/resume capabilities
- Implement comprehensive monitoring
- Enhance error recovery

### Phase 3: Production Features
- Add distributed session management
- Implement high availability
- Advanced resource optimization

## Implementation Details

### PID File Atomicity

To ensure PID files are written atomically and avoid corruption:

1. **Write to temporary file first**:
   ```
   Write PID to ./.autoware-mcp/instance_<port>_<pid>/sessions/<session>/.session.pid.tmp
   fsync() to ensure disk write
   rename() to final name (atomic operation)
   ```

2. **Use file locking**:
   ```
   Open with O_CREAT | O_EXCL to prevent races
   Use flock() for coordination between processes
   ```

3. **Include checksums**:
   ```
   Format: <PID>:<PGID>:<CHECKSUM>:<TIMESTAMP>
   Validate on read to detect corruption
   ```

### Process Group Creation

When starting a launch session:

1. **Create new process group**:
   ```
   Fork launch process
   Call setpgid(0, 0) in child to become group leader
   Record PGID in session.pgid file
   ```

2. **Propagate PGID to children**:
   ```
   Set PGID environment variable
   Child nodes inherit process group
   Enables single signal to entire tree
   ```

3. **Monitor group membership**:
   ```
   Periodically scan /proc/*/stat
   Verify PGID matches expected value
   Detect and log rogues processes
   ```

### Error Diagnostics Collection

The system maintains detailed error logs for debugging:

1. **Structured error format**:
   ```json
   {
     "timestamp": "2025-01-20T10:30:00Z",
     "session_id": "abc123",
     "node": "perception_node",
     "error_type": "launch_failure",
     "details": {
       "exit_code": 127,
       "stderr": "model file not found",
       "stdout_tail": "...",
       "environment": {...},
       "command": "ros2 run perception..."
     }
   }
   ```

2. **Error aggregation**:
   - Group similar errors
   - Track error frequencies
   - Identify patterns for AI learning

3. **Diagnostic bundles**:
   - Collect system state at error time
   - Include relevant log snippets
   - Package for offline analysis

## Multi-Instance Considerations

### User and Instance Isolation

The design supports multiple concurrent usage patterns:

1. **Multiple Users on Same System**:
   - Each user works in their own project directory
   - MCP runtime data stored in local `./.autoware-mcp/`
   - No conflicts between users' MCP servers
   - Process cleanup only affects own instances

2. **Multiple AI Agents per User**:
   - User can run multiple MCP servers on different ports
   - Each server gets unique `instance_<port>_<pid>/` directory
   - AI agents can work on same project without interference
   - Generated code shared via `./.autoware-mcp/generated/`

3. **Project Portability**:
   - All generated code stays with project
   - Can be committed to version control
   - Easy to share or move projects
   - Runtime data clearly separated from generated code

### Directory Structure Benefits

**For AI Development**:
- Generated files organized by type (launches/, nodes/, configs/)
- Versioning through incremental naming (v1, v2, v3...)
- Easy rollback to previous versions
- Clear separation of AI-generated vs human-written code

**For Operations**:
- Instance isolation prevents cross-contamination
- Stale instance detection and cleanup
- Comprehensive logging in project directory
- No need for system-wide permissions

**For Collaboration**:
- Generated code can be reviewed before commit
- Multiple agents can iterate on same codebase
- Clear audit trail of what was generated
- Integration with existing project workflows

## Conclusion

The proposed launch session management system provides comprehensive control over ROS2 launch files while maintaining system stability through robust PID/PGID tracking. The design specifically addresses multi-user and multi-agent scenarios by using project-local directories instead of system-wide paths.

Key design principles:
- **Instance isolation** via unique `instance_<port>_<pid>/` directories prevents conflicts
- **Project-local storage** keeps generated code with the project for version control
- **Persistent process tracking** via PID/PGID files ensures no orphaned processes
- **Multi-agent support** allows concurrent MCP servers in the same project
- **Atomic file operations** prevent corruption during crashes
- **Process group management** enables efficient bulk control
- **Detailed error diagnostics** support AI learning and improvement
- **Workflow-oriented tools** facilitate the generate-test-fix-deploy cycle

The cleanup mechanisms guarantee that even in catastrophic MCP server failures, all managed processes can be identified and terminated using the persistent PID/PGID files, while respecting instance boundaries. The project-local approach ensures that AI-generated code becomes a natural part of the development workflow, ready for version control and collaboration.

This architecture supports both immediate requirements and future scaling needs, making it suitable for production autonomous vehicle deployments where reliability, observability, and multi-tenancy are critical.