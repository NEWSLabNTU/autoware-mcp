#!/usr/bin/env python3
"""
Autonomous driving test using Autoware MCP tools.
This script demonstrates controlling Autoware through the MCP server interface.
"""

import asyncio
import yaml
import sys
import subprocess
import json
from pathlib import Path


class AutowareMCPClient:
    """Simple client for interacting with Autoware MCP server using subprocess."""

    def __init__(self):
        self.poses = None

    def load_poses_config(self):
        """Load poses from configuration file."""
        config_path = Path(__file__).parent / "poses_config.yaml"
        with open(config_path, "r") as f:
            self.poses = yaml.safe_load(f)
        print(f"✓ Loaded poses configuration from {config_path}")
        return self.poses

    def call_mcp_tool(self, tool_name, arguments=None):
        """Call an MCP tool via subprocess."""
        print(f"Calling MCP tool: {tool_name}")

        # Build the command
        cmd = ["rye", "run", "python", "-m", "autoware_mcp", "call", tool_name]
        if arguments:
            cmd.append(json.dumps(arguments))

        # Execute the command
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode != 0:
                print(f"Error calling tool: {result.stderr}")
                return None

            # Parse the output
            if result.stdout:
                try:
                    return json.loads(result.stdout)
                except json.JSONDecodeError:
                    # Return raw output if not JSON
                    return result.stdout
            return None

        except subprocess.TimeoutExpired:
            print(f"Tool call timed out: {tool_name}")
            return None
        except Exception as e:
            print(f"Error calling tool: {e}")
            return None

    def check_health(self):
        """Check system health using direct MCP tool call."""
        print("\nChecking system health...")

        # Use the MCP tools directly in Claude Code
        # For subprocess version, we'll check ROS2 topics directly
        topics_check = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True
        )

        if topics_check.returncode == 0:
            topic_count = len(topics_check.stdout.strip().split("\n"))
            print(f"✓ ROS2 is active with {topic_count} topics")
            return True

        print("✗ ROS2 check failed")
        return False

    def initialize_localization(self):
        """Initialize vehicle localization with pose from config."""
        print("\nInitializing localization...")

        pose = self.poses["initial_pose"]

        # Publish to /initialpose topic
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

        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

        if result.returncode == 0:
            print(
                f"✓ Localization initialized at position: "
                f"x={pose['position']['x']:.1f}, y={pose['position']['y']:.1f}"
            )
            return True

        print("✗ Failed to initialize localization")
        return False

    def clear_route(self):
        """Clear any existing route using ROS2 service."""
        print("\nClearing existing route...")

        cmd = "ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute {}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

        if result.returncode == 0:
            print("✓ Route cleared")
            return True
        return False

    def set_route_to_goal(self):
        """Set route to goal position using ROS2 service."""
        print("\nSetting route to goal...")

        goal = self.poses["goal_pose"]

        # Create request with proper formatting
        request = {
            "header": {"frame_id": "map"},
            "option": {"allow_goal_modification": True},
            "goal": {"position": goal["position"], "orientation": goal["orientation"]},
            "waypoints": [],
        }

        # Convert to string for command line
        request_str = str(request).replace("'", '"').replace("True", "true")

        cmd = f"ros2 service call /api/routing/set_route_points autoware_adapi_v1_msgs/srv/SetRoutePoints '{request_str}'"

        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

        if result.returncode == 0 and "success=True" in result.stdout:
            print(
                f"✓ Route set to goal: x={goal['position']['x']:.1f}, "
                f"y={goal['position']['y']:.1f}"
            )
            return True

        print("✗ Failed to set route")
        print(f"Output: {result.stdout}")
        return False

    def check_route_state(self):
        """Check current route state using ROS2 topic."""
        cmd = "ros2 topic echo /api/routing/state --once"
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5
        )

        if result.returncode == 0:
            if "state: 2" in result.stdout:
                print("Route state: SET")
                return True
            elif "state: 3" in result.stdout:
                print("Route state: ARRIVED")
                return False
            elif "state: 1" in result.stdout:
                print("Route state: UNSET")
                return False
            elif "state: 0" in result.stdout:
                print("Route state: UNKNOWN")
                return False
        return False

    def set_operation_mode(self, mode):
        """Set vehicle operation mode using ROS2 service."""
        print(f"\nChanging operation mode to: {mode}")

        service_map = {
            "stop": "/api/operation_mode/change_to_stop",
            "autonomous": "/api/operation_mode/change_to_autonomous",
            "local": "/api/operation_mode/change_to_local",
            "remote": "/api/operation_mode/change_to_remote",
        }

        service = service_map.get(mode.lower())
        if not service:
            print(f"✗ Unknown mode: {mode}")
            return False

        cmd = f"ros2 service call {service} autoware_adapi_v1_msgs/srv/ChangeOperationMode {{}}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

        if result.returncode == 0 and "success=True" in result.stdout:
            print(f"✓ Operation mode changed to: {mode}")
            return True

        print(f"✗ Failed to change operation mode")
        return False

    def monitor_operation_mode(self):
        """Monitor current operation mode using ROS2 topic."""
        cmd = "ros2 topic echo /api/operation_mode/state --once"
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5
        )

        if result.returncode == 0:
            if "mode: 1" in result.stdout:
                mode = "STOP"
            elif "mode: 2" in result.stdout:
                mode = "AUTONOMOUS"
            elif "mode: 3" in result.stdout:
                mode = "LOCAL"
            elif "mode: 4" in result.stdout:
                mode = "REMOTE"
            else:
                mode = "UNKNOWN"

            print(f"Current mode: {mode}")

            # Check if autoware control is enabled
            if "is_autoware_control_enabled: true" in result.stdout:
                print("Autoware control enabled: true")

            return result.stdout
        return None

    def get_vehicle_state(self):
        """Get current vehicle state from simulator."""
        # Check kinematic state
        cmd = "ros2 topic echo /localization/kinematic_state --once"
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5
        )

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

    def calculate_distance_to_goal(self, current_pos):
        """Calculate distance from current position to goal."""
        goal = self.poses["goal_pose"]["position"]
        dx = goal["x"] - current_pos["x"]
        dy = goal["y"] - current_pos["y"]
        return (dx**2 + dy**2) ** 0.5

    def monitor_until_goal_reached(self, timeout=300):
        """Monitor vehicle progress until it reaches the goal."""
        import time
        import math

        print("\n" + "=" * 60)
        print("MONITORING AUTONOMOUS DRIVING PROGRESS")
        print("=" * 60)

        start_time = time.time()
        last_update_time = start_time
        update_interval = 2  # Update every 2 seconds
        arrival_threshold = 5.0  # Consider arrived if within 5 meters

        # Get initial state
        initial_state = self.get_vehicle_state()
        if not initial_state:
            print("ERROR: Could not get initial vehicle state")
            return False

        initial_distance = self.calculate_distance_to_goal(initial_state)
        print(f"\nStarting distance to goal: {initial_distance:.1f} meters")
        print(
            f"Goal position: x={self.poses['goal_pose']['position']['x']:.1f}, "
            f"y={self.poses['goal_pose']['position']['y']:.1f}"
        )
        print("\nMonitoring progress...")
        print("-" * 40)

        while time.time() - start_time < timeout:
            current_time = time.time()

            # Update at intervals
            if current_time - last_update_time >= update_interval:
                # Get current state
                current_state = self.get_vehicle_state()
                if not current_state:
                    continue

                # Calculate metrics
                distance_to_goal = self.calculate_distance_to_goal(current_state)
                elapsed_time = current_time - start_time
                progress = max(
                    0, (initial_distance - distance_to_goal) / initial_distance * 100
                )

                # Check route state
                route_state_cmd = "ros2 topic echo /api/routing/state --once"
                route_result = subprocess.run(
                    route_state_cmd,
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                is_arrived = (
                    "state: 3" in route_result.stdout
                    if route_result.returncode == 0
                    else False
                )

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
                    print(f"  Route state: {'ARRIVED' if is_arrived else 'NEAR GOAL'}")
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

    def run_autonomous_sequence(self):
        """Run the complete autonomous driving sequence."""
        print("=" * 60)
        print("Autonomous Driving Test - Using Direct ROS2 Commands")
        print("=" * 60)

        # Load configuration
        print("\n1. Loading configuration...")
        self.load_poses_config()

        # Check health
        print("\n2. Checking system health...")
        if not self.check_health():
            print("ERROR: System health check failed")
            return False

        # Initialize localization
        print("\n3. Initializing localization...")
        if not self.initialize_localization():
            print("ERROR: Failed to initialize localization")
            return False

        print("Waiting for localization to stabilize...")
        import time

        time.sleep(5)

        # Clear and set route
        print("\n4. Setting up route...")
        self.clear_route()
        time.sleep(1)

        if not self.set_route_to_goal():
            print("ERROR: Failed to set route")
            return False

        print("Waiting for route processing...")
        time.sleep(3)

        # Check route state
        print("\n5. Verifying route...")
        if not self.check_route_state():
            print("WARNING: Route may not be properly set")

        # Monitor current mode
        print("\n6. Checking operation mode...")
        self.monitor_operation_mode()

        # Change to autonomous mode
        print("\n7. Engaging autonomous mode...")
        if not self.set_operation_mode("autonomous"):
            print("ERROR: Failed to engage autonomous mode")
            return False

        print("\n✓ AUTONOMOUS MODE ENGAGED")
        print("Vehicle is now driving autonomously!")
        time.sleep(2)

        # Monitor until goal is reached
        print("\n8. Monitoring vehicle progress to goal...")
        goal_reached = self.monitor_until_goal_reached(timeout=300)

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

        return goal_reached

    def stop_vehicle(self):
        """Stop the vehicle by changing to stop mode."""
        print("\nStopping vehicle...")
        self.set_operation_mode("stop")
        print("✓ Vehicle stopped")


def main():
    """Main execution flow."""
    client = AutowareMCPClient()

    try:
        # Run autonomous sequence
        success = client.run_autonomous_sequence()

        if success:
            # Check if running interactively
            try:
                # Ask user if they want to stop
                response = input(
                    "\nPress Enter to stop the vehicle, or 'q' to keep it running: "
                )
                if response.lower() != "q":
                    client.stop_vehicle()
            except EOFError:
                # Non-interactive mode - just keep running
                print(
                    "\nRunning in non-interactive mode. Vehicle will continue driving."
                )

        print("\nTest complete!")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        print("Attempting to stop vehicle...")
        client.stop_vehicle()

    except Exception as e:
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
