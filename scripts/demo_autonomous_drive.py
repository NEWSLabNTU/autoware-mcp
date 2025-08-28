#!/usr/bin/env python3
"""
Demo autonomous driving using fixed Autoware MCP tools.
"""

import time
import math
import yaml
import subprocess
import json
from pathlib import Path


def load_poses_config():
    """Load poses from configuration file."""
    config_path = Path(__file__).parent / "examples/poses_config.yaml"
    with open(config_path, "r") as f:
        poses = yaml.safe_load(f)
    print(f"✓ Loaded poses configuration")
    return poses


def call_mcp_tool(tool_name, arguments=None):
    """Call an MCP tool directly using Python import."""
    import sys

    sys.path.insert(0, "/home/aeon/repos/autoware-mcp/src")

    if tool_name == "health_check":
        from autoware_mcp.tools.system_tools import health_check

        return health_check()
    elif tool_name == "initialize_localization":
        from autoware_mcp.tools.localization_tools import initialize_localization

        return initialize_localization(**arguments)
    elif tool_name == "set_route":
        from autoware_mcp.tools.routing_tools import set_route

        return set_route(**arguments)
    elif tool_name == "get_current_route":
        from autoware_mcp.tools.routing_tools import get_current_route

        return get_current_route()
    elif tool_name == "set_operation_mode":
        from autoware_mcp.tools.operation_mode_tools import set_operation_mode

        return set_operation_mode(**arguments)
    elif tool_name == "get_vehicle_state":
        from autoware_mcp.tools.vehicle_tools import get_vehicle_state

        return get_vehicle_state()
    elif tool_name == "call_ros2_service":
        from autoware_mcp.tools.ros2_tools import call_ros2_service

        return call_ros2_service(**arguments)
    else:
        raise ValueError(f"Unknown tool: {tool_name}")


def calculate_distance(x1, y1, x2, y2):
    """Calculate 2D distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def main():
    print("=" * 60)
    print("AUTONOMOUS DRIVING DEMO - Using Fixed MCP Tools")
    print("=" * 60)

    # Load configuration
    print("\n1. Loading configuration...")
    poses = load_poses_config()

    initial_pose = poses["initial_pose"]
    goal_pose = poses["goal_pose"]

    # Calculate total distance
    total_distance = calculate_distance(
        initial_pose["position"]["x"],
        initial_pose["position"]["y"],
        goal_pose["position"]["x"],
        goal_pose["position"]["y"],
    )
    print(f"Total distance to goal: {total_distance:.2f} meters")

    # Check health
    print("\n2. Checking system health...")
    health = call_mcp_tool("health_check")
    if health["status"] == "healthy":
        print("✓ System is healthy")
    else:
        print("✗ System health check failed")
        return False

    # Initialize localization
    print("\n3. Initializing localization...")
    loc_result = call_mcp_tool(
        "initialize_localization",
        {
            "pose": {
                "position": initial_pose["position"],
                "orientation": initial_pose["orientation"],
            }
        },
    )
    if loc_result["success"]:
        print(f"✓ Localization initialized")
    else:
        print("✗ Failed to initialize localization")
        return False

    print("Waiting for localization to stabilize...")
    time.sleep(5)

    # Clear and set route
    print("\n4. Setting up route...")

    # Clear any existing route
    clear_result = call_mcp_tool(
        "call_ros2_service",
        {
            "service_name": "/api/routing/clear_route",
            "service_type": "autoware_adapi_v1_msgs/srv/ClearRoute",
            "request": {},
        },
    )
    print("✓ Route cleared")
    time.sleep(1)

    # Set route using fixed MCP tool
    route_result = call_mcp_tool(
        "set_route",
        {
            "goal_pose": {
                "position": goal_pose["position"],
                "orientation": goal_pose["orientation"],
            },
            "option": {"allow_goal_modification": True},
        },
    )

    if route_result.get("success"):
        print(f"✓ Route set to goal")
    else:
        print(f"✗ Failed to set route: {route_result.get('message')}")
        return False

    print("Waiting for route processing...")
    time.sleep(3)

    # Check route state
    print("\n5. Verifying route...")
    route_state = call_mcp_tool("get_current_route")
    if route_state.get("state") == 2:
        print("✓ Route state: SET (ready to drive)")
    else:
        print(f"⚠ Route state: {route_state.get('state')}")

    # Change to autonomous mode
    print("\n6. Engaging autonomous mode...")
    mode_result = call_mcp_tool("set_operation_mode", {"mode": "autonomous"})
    if mode_result.get("success"):
        print("✓ AUTONOMOUS MODE ENGAGED")
        print("Vehicle is now driving autonomously!")
    else:
        print("✗ Failed to engage autonomous mode")
        return False

    time.sleep(2)

    # Monitor progress
    print("\n7. Monitoring vehicle progress...")
    print("-" * 40)

    start_time = time.time()
    last_update_time = start_time
    stuck_counter = 0

    while True:
        try:
            current_time = time.time()
            elapsed_time = current_time - start_time

            # Update every 2 seconds
            if current_time - last_update_time >= 2:
                # Get vehicle state
                vehicle_state = call_mcp_tool("get_vehicle_state")
                route_state = call_mcp_tool("get_current_route")

                if vehicle_state and "kinematics" in vehicle_state:
                    kin = vehicle_state["kinematics"]

                    # Get position
                    if "position" in kin:
                        current_x = kin["position"]["x"]
                        current_y = kin["position"]["y"]
                    else:
                        print("Waiting for vehicle position...")
                        time.sleep(2)
                        continue

                    # Calculate speed
                    vel_x = kin.get("linear_velocity", {}).get("x", 0)
                    vel_y = kin.get("linear_velocity", {}).get("y", 0)
                    speed = math.sqrt(vel_x**2 + vel_y**2)

                    # Calculate distance to goal
                    distance_to_goal = calculate_distance(
                        current_x,
                        current_y,
                        goal_pose["position"]["x"],
                        goal_pose["position"]["y"],
                    )

                    # Calculate progress
                    distance_traveled = total_distance - distance_to_goal
                    progress = (
                        (distance_traveled / total_distance) * 100
                        if total_distance > 0
                        else 0
                    )

                    # Print status
                    print(
                        f"[{elapsed_time:5.1f}s] Speed: {speed:.2f} m/s | "
                        f"Distance: {distance_to_goal:.1f}m | Progress: {progress:.1f}%"
                    )

                    # Check if arrived
                    if route_state and route_state.get("state") == 3:
                        print("-" * 40)
                        print("\n✓ ARRIVED at goal!")
                        break

                    if distance_to_goal < 5.0:
                        print("-" * 40)
                        print(
                            f"\n✓ Vehicle reached goal! Final distance: {distance_to_goal:.2f} m"
                        )
                        break

                    # Check if stuck
                    if speed < 0.1:
                        stuck_counter += 1
                        if stuck_counter > 5:
                            print("⚠ Vehicle may be stuck (speed < 0.1 m/s)")
                    else:
                        stuck_counter = 0

                    last_update_time = current_time

                # Timeout after 5 minutes
                if elapsed_time > 300:
                    print("\n⚠ Timeout: Did not reach goal within 5 minutes")
                    break

            time.sleep(0.5)

        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
            break
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(2)

    print("\n" + "=" * 60)
    print("AUTONOMOUS DRIVING DEMO COMPLETE")
    print("=" * 60)

    return True


if __name__ == "__main__":
    main()
