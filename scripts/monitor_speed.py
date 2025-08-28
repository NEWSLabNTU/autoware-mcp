#!/usr/bin/env python3
import time
import math
import subprocess


def get_vehicle_position_speed():
    """Get current vehicle position and speed from kinematic state."""
    cmd = "ros2 topic echo /localization/kinematic_state --once"
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0:
            lines = result.stdout.split("\n")
            pos_x, pos_y = 0, 0
            vel_x, vel_y = 0, 0

            in_position = False
            in_linear = False

            for i, line in enumerate(lines):
                line = line.strip()

                if "position:" in line:
                    in_position = True
                    continue

                if in_position and line.startswith("x:"):
                    pos_x = float(line.split(":")[1].strip())
                elif in_position and line.startswith("y:"):
                    pos_y = float(line.split(":")[1].strip())
                    in_position = False

                if "linear:" in line:
                    in_linear = True
                    continue

                if in_linear and line.startswith("x:"):
                    vel_x = float(line.split(":")[1].strip())
                elif in_linear and line.startswith("y:"):
                    vel_y = float(line.split(":")[1].strip())
                    in_linear = False

            speed = math.sqrt(vel_x**2 + vel_y**2)
            return pos_x, pos_y, speed
    except:
        pass
    return None, None, None


def get_route_state():
    """Get current route state."""
    cmd = "ros2 topic echo /api/routing/state --once"
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0:
            if "state: 3" in result.stdout:
                return 3  # ARRIVED
            elif "state: 2" in result.stdout:
                return 2  # SET
            elif "state: 1" in result.stdout:
                return 1  # UNSET
    except:
        pass
    return 0


def main():
    goal_x = 3758.956298828125
    goal_y = 73689.2890625
    initial_x = 3752.342041015625
    initial_y = 73736.09375

    total_distance = math.sqrt((goal_x - initial_x) ** 2 + (goal_y - initial_y) ** 2)

    print("=" * 60)
    print("AUTONOMOUS DRIVING IN PROGRESS")
    print(f"Total distance to goal: {total_distance:.2f} meters")
    print("=" * 60)
    print()

    start_time = time.time()
    last_update = start_time
    update_interval = 2.0

    while True:
        current_time = time.time()

        if current_time - last_update >= update_interval:
            elapsed = current_time - start_time

            # Get vehicle state
            pos_x, pos_y, speed = get_vehicle_position_speed()
            if pos_x is None:
                print(f"[{elapsed:5.1f}s] Waiting for vehicle data...")
                last_update = current_time
                continue

            # Calculate distance to goal
            distance_to_goal = math.sqrt((goal_x - pos_x) ** 2 + (goal_y - pos_y) ** 2)

            # Calculate progress
            distance_traveled = total_distance - distance_to_goal
            progress = (
                (distance_traveled / total_distance * 100) if total_distance > 0 else 0
            )

            # Get route state
            route_state = get_route_state()
            state_str = {0: "UNKNOWN", 1: "UNSET", 2: "DRIVING", 3: "ARRIVED"}[
                route_state
            ]

            # Print status
            print(
                f"[{elapsed:5.1f}s] Speed: {speed:4.2f} m/s | Distance: {distance_to_goal:5.1f}m | Progress: {progress:5.1f}% | State: {state_str}"
            )

            # Check if arrived
            if route_state == 3 or distance_to_goal < 5.0:
                print()
                print("=" * 60)
                print("✓ GOAL REACHED!")
                print(f"Final distance: {distance_to_goal:.2f} meters")
                print(f"Total time: {elapsed:.1f} seconds")
                print("=" * 60)
                break

            # Check if stuck
            if speed < 0.1 and elapsed > 30:
                print("  ⚠ Vehicle speed is very low")

            last_update = current_time

            # Timeout after 5 minutes
            if elapsed > 300:
                print("\n⚠ Timeout: 5 minutes elapsed")
                break

        time.sleep(0.5)


if __name__ == "__main__":
    main()
