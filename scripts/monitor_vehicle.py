#!/usr/bin/env python3
import time
import math
import subprocess
import json


def get_vehicle_state():
    """Get current vehicle state from MCP"""
    try:
        result = subprocess.run(
            [
                "python",
                "-c",
                """
import sys
sys.path.insert(0, '/home/aeon/repos/autoware-mcp/src')
from autoware_mcp.tools.vehicle_tools import get_vehicle_state
state = get_vehicle_state()
import json
print(json.dumps(state))
""",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            return json.loads(result.stdout)
    except:
        pass
    return None


def get_route_state():
    """Get current route state from MCP"""
    try:
        result = subprocess.run(
            [
                "python",
                "-c",
                """
import sys
sys.path.insert(0, '/home/aeon/repos/autoware-mcp/src')
from autoware_mcp.tools.routing_tools import get_current_route
route = get_current_route()
import json
print(json.dumps(route))
""",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            return json.loads(result.stdout)
    except:
        pass
    return None


def calculate_distance(x1, y1, x2, y2):
    """Calculate 2D distance between two points"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def main():
    goal_x = 3758.956298828125
    goal_y = 73689.2890625
    initial_x = 3752.342041015625
    initial_y = 73736.09375

    total_distance = calculate_distance(initial_x, initial_y, goal_x, goal_y)
    print(f"Total distance to goal: {total_distance:.2f} meters")
    print("\nStarting autonomous driving...")
    print("-" * 60)

    stuck_counter = 0
    last_update_time = time.time()

    while True:
        try:
            # Get vehicle state
            vehicle_state = get_vehicle_state()
            route_state = get_route_state()

            if vehicle_state and "kinematics" in vehicle_state:
                kin = vehicle_state["kinematics"]
                if "position" in kin:
                    current_x = kin["position"]["x"]
                    current_y = kin["position"]["y"]
                else:
                    print("Waiting for vehicle position...")
                    time.sleep(2)
                    continue

                # Calculate speed from velocities
                vel_x = kin.get("linear_velocity", {}).get("x", 0)
                vel_y = kin.get("linear_velocity", {}).get("y", 0)
                speed = math.sqrt(vel_x**2 + vel_y**2)

                # Calculate distance to goal
                distance_to_goal = calculate_distance(
                    current_x, current_y, goal_x, goal_y
                )

                # Calculate progress
                distance_traveled = total_distance - distance_to_goal
                progress = (
                    (distance_traveled / total_distance) * 100
                    if total_distance > 0
                    else 0
                )

                # Print status
                current_time = time.time()
                if current_time - last_update_time >= 2:  # Update every 2 seconds
                    print(f"Time: {time.strftime('%H:%M:%S')}")
                    print(
                        f"Speed: {speed:.2f} m/s | Distance to goal: {distance_to_goal:.2f} m | Progress: {progress:.1f}%"
                    )

                    if route_state:
                        route_status = route_state.get("state", 0)
                        if route_status == 3:
                            print("\n✓ ARRIVED at goal!")
                            break

                    if speed < 0.1:
                        stuck_counter += 1
                        if stuck_counter > 5:
                            print("⚠ Vehicle may be stuck (speed < 0.1 m/s)")
                    else:
                        stuck_counter = 0

                    print("-" * 60)
                    last_update_time = current_time

                # Check if arrived
                if distance_to_goal < 5.0:
                    print(
                        f"\n✓ Vehicle reached goal! Final distance: {distance_to_goal:.2f} m"
                    )
                    break

            time.sleep(0.5)

        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
            break
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(2)


if __name__ == "__main__":
    main()
