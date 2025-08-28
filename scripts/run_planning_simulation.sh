#!/usr/bin/env bash

# Planning simulation management script
# Usage: ./run_planning_simulation.sh [start|stop|restart|status]

PIDFILE="/tmp/planning_simulation.pid"
LOGFILE="/tmp/planning_simulation.log"

# Configuration
MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"
VEHICLE_MODEL="${VEHICLE_MODEL:-sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-sample_sensor_kit}"
DISPLAY="${DISPLAY:-:1}"

function start_simulation() {
    if [ -f "$PIDFILE" ]; then
        PID=$(cat "$PIDFILE")
        if ps -p "$PID" > /dev/null 2>&1; then
            echo "Planning simulation is already running (PID: $PID)"
            return 1
        else
            echo "Removing stale PID file"
            rm -f "$PIDFILE"
        fi
    fi
    
    echo "Starting planning simulation..."
    echo "Map path: $MAP_PATH"
    echo "Vehicle model: $VEHICLE_MODEL"
    echo "Sensor model: $SENSOR_MODEL"
    echo "Display: $DISPLAY"
    echo "Log file: $LOGFILE"
    
    # Start in a new process group so we can kill the entire group later
    DISPLAY="$DISPLAY" nohup setsid ros2 launch autoware_launch planning_simulator.launch.xml \
        map_path:="$MAP_PATH" \
        vehicle_model:="$VEHICLE_MODEL" \
        sensor_model:="$SENSOR_MODEL" \
        > "$LOGFILE" 2>&1 &
    
    PID=$!
    echo "$PID" > "$PIDFILE"
    
    # Wait a bit to check if it started successfully
    sleep 3
    
    if ps -p "$PID" > /dev/null 2>&1; then
        echo "Planning simulation started successfully (PID: $PID)"
        return 0
    else
        echo "Failed to start planning simulation"
        rm -f "$PIDFILE"
        tail -20 "$LOGFILE"
        return 1
    fi
}

function stop_simulation() {
    if [ ! -f "$PIDFILE" ]; then
        echo "Planning simulation is not running (no PID file found)"
        return 1
    fi
    
    PID=$(cat "$PIDFILE")
    
    if ! ps -p "$PID" > /dev/null 2>&1; then
        echo "Planning simulation is not running (process not found)"
        rm -f "$PIDFILE"
        return 1
    fi
    
    echo "Stopping planning simulation (PID: $PID)..."
    
    # Get the process group ID (PGID) to kill the entire process tree
    PGID=$(ps -o pgid= -p "$PID" 2>/dev/null | tr -d ' ')
    
    if [ -n "$PGID" ]; then
        echo "Killing process group: $PGID"
        # Kill the entire process group
        kill -TERM -"$PGID" 2>/dev/null
        
        # Wait for processes to terminate
        for i in {1..10}; do
            if ! ps -p "$PID" > /dev/null 2>&1; then
                echo "Process group terminated successfully"
                break
            fi
            sleep 1
        done
        
        # Force kill the entire process group if still running
        if ps -p "$PID" > /dev/null 2>&1; then
            echo "Force killing process group..."
            kill -9 -"$PGID" 2>/dev/null
            sleep 2
        fi
    else
        # Fallback: try to kill just the main process
        echo "Could not find process group, trying individual process..."
        kill -TERM "$PID" 2>/dev/null
        sleep 2
        kill -9 "$PID" 2>/dev/null
    fi
    
    # Additional cleanup for any remaining ROS2 processes
    echo "Cleaning up remaining ROS2 processes..."
    pkill -f "autoware_launch.*planning_simulator" 2>/dev/null
    pkill -f "component_container" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    
    # Stop and restart ROS2 daemon to clean up any remaining nodes
    echo "Restarting ROS2 daemon to ensure cleanup..."
    ros2 daemon stop 2>/dev/null
    sleep 1
    ros2 daemon start 2>/dev/null
    
    rm -f "$PIDFILE"
    echo "Planning simulation stopped"
    return 0
}

function restart_simulation() {
    echo "Restarting planning simulation..."
    stop_simulation
    sleep 2
    start_simulation
    return $?
}

function wait_for_ready() {
    echo "Waiting for Autoware to be fully loaded..."
    local timeout=${1:-180}  # Default 3 minutes timeout
    local start_time=$(date +%s)
    local elapsed=0
    
    # Phase 1: Wait for basic nodes to start
    echo "Phase 1: Waiting for nodes to start..."
    while [ $elapsed -lt $timeout ]; do
        local node_count=$(ros2 node list 2>/dev/null | wc -l)
        elapsed=$(($(date +%s) - start_time))
        
        if [ $node_count -gt 30 ]; then
            echo "  [$elapsed s] ✓ $node_count nodes are running"
            break
        else
            echo "  [$elapsed s] $node_count nodes running, waiting for more..."
            sleep 3
        fi
    done
    
    # Phase 2: Wait for critical topics
    echo "Phase 2: Checking critical topics..."
    local critical_topics=(
        "/map/vector_map"
        "/map/pointcloud_map"
        "/api/operation_mode/state"
        "/api/routing/state"
        "/tf"
        "/tf_static"
    )
    
    for topic in "${critical_topics[@]}"; do
        echo -n "  Checking $topic..."
        local retries=0
        while [ $retries -lt 10 ] && [ $elapsed -lt $timeout ]; do
            if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
                echo " ✓"
                break
            fi
            retries=$((retries + 1))
            elapsed=$(($(date +%s) - start_time))
            sleep 2
        done
        if [ $retries -eq 10 ]; then
            echo " ✗ (not ready)"
        fi
    done
    
    # Phase 3: Check if map is actually loaded
    echo "Phase 3: Verifying map data..."
    local map_loaded=false
    while [ $elapsed -lt $timeout ]; do
        elapsed=$(($(date +%s) - start_time))
        
        # Check if vector map has data
        if timeout 2 ros2 topic echo /map/vector_map --once 2>/dev/null | grep -q "data:"; then
            echo "  [$elapsed s] ✓ Vector map has data"
            map_loaded=true
            break
        else
            echo "  [$elapsed s] Waiting for map data..."
            sleep 5
        fi
    done
    
    if [ "$map_loaded" = true ]; then
        echo "✓ Autoware is fully loaded and ready after $elapsed seconds"
        # Give it a bit more time to stabilize
        echo "  Waiting 5 more seconds for stabilization..."
        sleep 5
        return 0
    else
        echo "✗ Autoware failed to fully load within timeout"
        return 1
    fi
}

function status_simulation() {
    if [ ! -f "$PIDFILE" ]; then
        echo "Planning simulation is not running (no PID file)"
        return 1
    fi
    
    PID=$(cat "$PIDFILE")
    
    if ps -p "$PID" > /dev/null 2>&1; then
        # Get process group information
        PGID=$(ps -o pgid= -p "$PID" 2>/dev/null | tr -d ' ')
        CHILD_COUNT=$(pgrep -g "$PGID" 2>/dev/null | wc -l)
        
        echo "Planning simulation is running (PID: $PID, PGID: $PGID)"
        echo "Process group has $CHILD_COUNT child processes"
        
        # Check ROS2 nodes
        echo ""
        echo "Checking ROS2 status..."
        NODE_COUNT=$(timeout 5 ros2 node list 2>/dev/null | wc -l)
        echo "Active ROS2 nodes: $NODE_COUNT"
        if [ "$NODE_COUNT" -gt 0 ]; then
            echo "First 10 nodes:"
            timeout 5 ros2 node list 2>/dev/null | head -10
        fi
        
        return 0
    else
        echo "Planning simulation is not running (process not found)"
        rm -f "$PIDFILE"
        return 1
    fi
}

function show_logs() {
    if [ -f "$LOGFILE" ]; then
        echo "Last 50 lines of planning simulation log:"
        tail -50 "$LOGFILE"
    else
        echo "No log file found"
    fi
}

# Main script
case "$1" in
    start)
        start_simulation
        if [ $? -eq 0 ]; then
            wait_for_ready
        fi
        ;;
    stop)
        stop_simulation
        ;;
    restart)
        restart_simulation
        if [ $? -eq 0 ]; then
            wait_for_ready
        fi
        ;;
    wait)
        wait_for_ready ${2:-180}
        ;;
    status)
        status_simulation
        ;;
    logs)
        show_logs
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|wait|logs}"
        echo ""
        echo "Commands:"
        echo "  start   - Start the planning simulation and wait for it to be ready"
        echo "  stop    - Stop the planning simulation"
        echo "  restart - Restart the planning simulation and wait for it to be ready"
        echo "  status  - Check if the simulation is running"
        echo "  wait [timeout] - Wait for Autoware to be fully loaded (default: 180s)"
        echo "  logs    - Show the last 50 lines of the simulation log"
        echo ""
        echo "Environment variables:"
        echo "  MAP_PATH      - Path to map directory (default: \$HOME/autoware_map/sample-map-planning)"
        echo "  VEHICLE_MODEL - Vehicle model to use (default: sample_vehicle)"
        echo "  SENSOR_MODEL  - Sensor model to use (default: sample_sensor_kit)"
        echo "  DISPLAY       - X11 display to use (default: :1)"
        exit 1
        ;;
esac

exit $?