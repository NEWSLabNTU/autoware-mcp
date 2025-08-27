#!/usr/bin/env python3
"""
Autonomous driving test using direct ROS2 service calls.
This script demonstrates controlling Autoware directly through ROS2 services.
"""

import subprocess
import time
import yaml
import sys
from pathlib import Path


def load_poses_config():
    """Load poses from configuration file."""
    config_path = Path(__file__).parent / "poses_config.yaml"
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def call_ros2_service(service_name, service_type, request_data):
    """Call a ROS2 service directly using subprocess."""
    # Convert request data to string format for ros2 service call
    request_str = str(request_data).replace("'", '"')

    cmd = ["ros2", "service", "call", service_name, service_type, request_str]

    print(f"Calling service: {service_name}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Error calling service: {result.stderr}")
        return False

    print(f"Response: {result.stdout}")
    return True


def publish_initial_pose(pose_data):
    """Publish initial pose using ros2 topic pub."""
    pose = pose_data["initial_pose"]

    cmd = f'''ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: {pose["position"]["x"]}
      y: {pose["position"]["y"]}
      z: {pose["position"]["z"]}
    orientation:
      x: {pose["orientation"]["x"]}
      y: {pose["orientation"]["y"]}
      z: {pose["orientation"]["z"]}
      w: {pose["orientation"]["w"]}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"'''

    print("Publishing initial pose...")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Error publishing initial pose: {result.stderr}")
        return False

    print("✓ Initial pose published")
    return True


def set_route_to_goal(pose_data):
    """Set route to goal using ROS2 service call."""
    goal = pose_data["goal_pose"]

    request = {
        "header": {"frame_id": "map"},
        "option": {"allow_goal_modification": True},
        "goal": {
            "position": {
                "x": goal["position"]["x"],
                "y": goal["position"]["y"],
                "z": goal["position"]["z"],
            },
            "orientation": {
                "x": goal["orientation"]["x"],
                "y": goal["orientation"]["y"],
                "z": goal["orientation"]["z"],
                "w": goal["orientation"]["w"],
            },
        },
        "waypoints": [],
    }

    return call_ros2_service(
        "/api/routing/set_route_points",
        "autoware_adapi_v1_msgs/srv/SetRoutePoints",
        request,
    )


def clear_route():
    """Clear the current route."""
    return call_ros2_service(
        "/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute", "{}"
    )


def set_operation_mode(mode):
    """Set vehicle operation mode."""
    request = {
        "mode": mode  # 1=STOP, 2=AUTONOMOUS, 3=LOCAL, 4=REMOTE
    }

    mode_map = {"stop": 1, "autonomous": 2, "local": 3, "remote": 4}
    mode_num = mode_map.get(mode.lower(), 1)

    return call_ros2_service(
        "/api/operation_mode/change_to_" + mode.lower(),
        "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
        "{}",
    )


def check_autoware_ready():
    """Check if Autoware is ready by checking key topics."""
    topics_to_check = [
        "/map/vector_map",
        "/api/operation_mode/state",
        "/api/routing/state",
    ]

    for topic in topics_to_check:
        cmd = f"ros2 topic info {topic}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if "Publisher count: 0" in result.stdout:
            print(f"✗ Topic {topic} has no publishers")
            return False

    print("✓ Autoware appears ready")
    return True


def get_vehicle_state():
    """Get current vehicle state from simulator."""
    # Check kinematic state
    cmd = "ros2 topic echo /localization/kinematic_state --once"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)

    if result.returncode == 0:
        # Parse position and velocity from output
        lines = result.stdout.split("\n")
        pos_x, pos_y, pos_z = 0, 0, 0
        vel_x, vel_y = 0, 0

        # Track if we're in pose or twist section
        in_pose = False
        in_twist = False
        in_linear = False

        for i, line in enumerate(lines):
            line = line.strip()

            # Check for pose section
            if line == "pose:":
                in_pose = True
                in_twist = False
                continue

            # Check for twist section
            elif line == "twist:":
                in_twist = True
                in_pose = False
                continue

            # Parse position from pose section
            if in_pose and "position:" in line:
                for j in range(1, 4):
                    if i + j < len(lines):
                        next_line = lines[i + j].strip()
                        if next_line.startswith("x:"):
                            pos_x = float(next_line.split(":")[1].strip())
                        elif next_line.startswith("y:"):
                            pos_y = float(next_line.split(":")[1].strip())
                        elif next_line.startswith("z:"):
                            pos_z = float(next_line.split(":")[1].strip())

            # Parse velocity from twist section
            if in_twist and "linear:" in line:
                in_linear = True
                for j in range(1, 3):
                    if i + j < len(lines):
                        next_line = lines[i + j].strip()
                        if next_line.startswith("x:"):
                            vel_x = float(next_line.split(":")[1].strip())
                        elif next_line.startswith("y:"):
                            vel_y = float(next_line.split(":")[1].strip())

        speed = (vel_x**2 + vel_y**2) ** 0.5
        return {"x": pos_x, "y": pos_y, "z": pos_z, "speed": speed}

    print("Could not get vehicle state")
    return None


def calculate_distance_to_goal(current_pos, goal_pos):
    """Calculate distance from current position to goal."""
    dx = goal_pos["x"] - current_pos["x"]
    dy = goal_pos["y"] - current_pos["y"]
    return (dx**2 + dy**2) ** 0.5


def check_route_state():
    """Check current route state using ROS2 topic."""
    cmd = "ros2 topic echo /api/routing/state --once"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)

    if result.returncode == 0:
        if "state: 2" in result.stdout:
            return "SET"
        elif "state: 3" in result.stdout:
            return "ARRIVED"
        elif "state: 1" in result.stdout:
            return "UNSET"
        elif "state: 0" in result.stdout:
            return "UNKNOWN"
    return "ERROR"


def monitor_until_goal_reached(goal_pos, timeout=300):
    """Monitor vehicle progress until it reaches the goal."""
    print("\n" + "=" * 60)
    print("MONITORING AUTONOMOUS DRIVING PROGRESS")
    print("=" * 60)

    start_time = time.time()
    last_update_time = start_time
    update_interval = 2  # Update every 2 seconds
    arrival_threshold = 5.0  # Consider arrived if within 5 meters

    # Get initial state
    initial_state = get_vehicle_state()
    if not initial_state:
        print("ERROR: Could not get initial vehicle state")
        return False

    initial_distance = calculate_distance_to_goal(initial_state, goal_pos)
    print(f"\nStarting distance to goal: {initial_distance:.1f} meters")
    print(f"Goal position: x={goal_pos['x']:.1f}, y={goal_pos['y']:.1f}")
    print("\nMonitoring progress...")
    print("-" * 40)

    while time.time() - start_time < timeout:
        current_time = time.time()

        # Update at intervals
        if current_time - last_update_time >= update_interval:
            # Get current state
            current_state = get_vehicle_state()
            if not current_state:
                continue

            # Calculate metrics
            distance_to_goal = calculate_distance_to_goal(current_state, goal_pos)
            elapsed_time = current_time - start_time
            progress = max(
                0, (initial_distance - distance_to_goal) / initial_distance * 100
            )

            # Check route state
            route_state = check_route_state()
            is_arrived = route_state == "ARRIVED"

            # Print status
            print(
                f"[{elapsed_time:5.1f}s] Pos: ({current_state['x']:.1f}, {current_state['y']:.1f}) | "
                f"Speed: {current_state['speed']:.1f} m/s | "
                f"Distance: {distance_to_goal:.1f}m | "
                f"Progress: {progress:.1f}%"
            )

            # Check if arrived
            if is_arrived or distance_to_goal < arrival_threshold:
                print("-" * 40)
                print(f"\n✓ GOAL REACHED!")
                print(
                    f"  Final position: ({current_state['x']:.1f}, {current_state['y']:.1f})"
                )
                print(f"  Distance to goal: {distance_to_goal:.1f} meters")
                print(f"  Total time: {elapsed_time:.1f} seconds")
                print(f"  Route state: {route_state}")
                return True

            # Check if vehicle is stuck
            if (
                current_state["speed"] < 0.1
                and elapsed_time > 30
                and distance_to_goal > 10
            ):
                print("\n⚠ WARNING: Vehicle appears to be stuck (speed < 0.1 m/s)")

            last_update_time = current_time

        time.sleep(0.5)

    print("\n✗ TIMEOUT: Did not reach goal within {} seconds".format(timeout))
    return False


def main():
    """Main execution flow for autonomous driving using direct ROS2 calls."""
    print("=" * 60)
    print("Autonomous Driving Test - Direct ROS2 Service Calls")
    print("=" * 60)

    # Load poses configuration
    print("\n1. Loading poses configuration...")
    poses = load_poses_config()
    print(f"✓ Loaded initial pose and goal pose from config")

    # Check if Autoware is ready
    print("\n2. Checking Autoware status...")
    if not check_autoware_ready():
        print(
            "ERROR: Autoware is not ready. Please start the planning simulation first."
        )
        print("Run: ./run_planning_simulation.sh start")
        return 1

    # Initialize localization
    print("\n3. Initializing localization...")
    if not publish_initial_pose(poses):
        print("ERROR: Failed to set initial pose")
        return 1

    print("Waiting for localization to stabilize...")
    time.sleep(5)

    # Clear any existing route
    print("\n4. Clearing any existing route...")
    clear_route()
    time.sleep(1)

    # Set route to goal
    print("\n5. Setting route to goal...")
    print(
        f"Goal position: x={poses['goal_pose']['position']['x']:.1f}, "
        f"y={poses['goal_pose']['position']['y']:.1f}"
    )

    if not set_route_to_goal(poses):
        print("ERROR: Failed to set route")
        return 1

    print("Waiting for route to be processed...")
    time.sleep(3)

    # Verify route state
    print("\n6. Verifying route...")
    route_state = check_route_state()
    print(f"Route state: {route_state}")
    if route_state != "SET":
        print("WARNING: Route may not be properly set")

    # Change to autonomous mode
    print("\n7. Changing to autonomous mode...")
    if not set_operation_mode("autonomous"):
        print("ERROR: Failed to change to autonomous mode")
        return 1

    print("\n✓ AUTONOMOUS MODE ENGAGED")
    print("Vehicle is now driving autonomously!")
    time.sleep(2)

    # Monitor until goal is reached
    print("\n8. Monitoring vehicle progress to goal...")
    goal_reached = monitor_until_goal_reached(
        poses["goal_pose"]["position"], timeout=300
    )

    if goal_reached:
        print("\n" + "=" * 60)
        print("✓ AUTONOMOUS DRIVING MISSION COMPLETE")
        print("The vehicle has successfully reached its destination!")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("⚠ AUTONOMOUS DRIVING INCOMPLETE")
        print("The vehicle did not reach the goal within the timeout period.")
        print("=" * 60)

    # Option to stop
    try:
        response = input(
            "\nPress Enter to stop the vehicle, or 'q' to quit without stopping: "
        )
        if response.lower() != "q":
            print("Changing to stop mode...")
            set_operation_mode("stop")
            print("✓ Vehicle stopped")
    except EOFError:
        # Non-interactive mode - just keep running
        print("\nRunning in non-interactive mode. Vehicle will continue driving.")

    print("\nTest complete!")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        print("Attempting to stop vehicle...")
        set_operation_mode("stop")
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)
