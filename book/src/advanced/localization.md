# Localization Initialization Guide

## Overview

This document describes the correct method for initializing vehicle localization in Autoware's planning simulator, particularly when using RViz2 for visualization.

## Key Discovery

The planning simulator requires initial pose to be published to the `/initialpose` topic using the exact format that RViz2's "2D Pose Estimate" tool uses. This ensures proper initialization of the localization system and visualization of the vehicle in RViz2.

## Message Format

The initial pose must be published as a `geometry_msgs/msg/PoseWithCovarianceStamped` message with specific requirements:

### Required Fields

1. **Header**
   - `frame_id`: Must be set to `'map'` 
   - `stamp`: Can be zero (sec: 0, nanosec: 0)

2. **Pose**
   - `position`: x, y, z coordinates in map frame
   - `orientation`: Quaternion (x, y, z, w) representing vehicle orientation

3. **Covariance**
   - 36-element array representing 6x6 covariance matrix
   - Default values that work: `[0.25, 0.0, ..., 0.06853892326654787]`

## Implementation Methods

### Method 1: ROS2 Command Line (YAML Format)

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 3813.96
      y: 73729.3
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.6644
      w: 0.7473
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"
```

### Method 2: MCP Server API

The Autoware MCP server's `initialize_localization` function automatically publishes to both `/initialpose` and `/initialpose3d` topics with the correct format.

### Method 3: Python Script

```python
import subprocess
import json

def set_initial_pose(position, orientation):
    """Set initial pose using RViz 2D Pose Estimate format."""
    pose_cmd = f'''ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: {position['x']}
      y: {position['y']}
      z: {position['z']}
    orientation:
      x: {orientation['x']}
      y: {orientation['y']}
      z: {orientation['z']}
      w: {orientation['w']}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"'''
    
    result = subprocess.run(pose_cmd, shell=True, capture_output=True, text=True)
    return result.returncode == 0
```

## Verification

After setting the initial pose, verify localization is initialized:

1. **Check TF transforms**:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```
   This should show valid transforms between map and base_link frames.

2. **Check localization state topic**:
   ```bash
   ros2 topic echo /api/localization/initialization_state
   ```
   The state should be `2` (INITIALIZED).

3. **Visual verification in RViz2**:
   - The vehicle model should appear at the specified position
   - The "Localization" panel should show "Initialized" status

## Important Notes

1. **Topic Priority**: While both `/initialpose` and `/initialpose3d` can be used, `/initialpose` is more reliable for planning simulator as it matches RViz2's behavior exactly.

2. **Frame ID**: The `frame_id` must be set to `'map'` - this is critical for proper localization.

3. **YAML vs JSON**: When using `ros2 topic pub`, use YAML format (as shown above) rather than JSON format for better compatibility.

4. **Covariance Values**: The specific covariance values (especially the last element 0.06853892326654787) match what RViz2 uses internally and ensure proper uncertainty representation.

## Troubleshooting

If localization fails to initialize:

1. Ensure the planning simulator is fully started (wait 10-15 seconds after launch)
2. Check that the map frame exists: `ros2 run tf2_tools view_frames`
3. Verify the topic is available: `ros2 topic list | grep initialpose`
4. Check for any error messages in the simulator logs
5. Try using RViz2's "2D Pose Estimate" tool manually to confirm the system works

## References

- [Autoware Documentation - Localization](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/creating-vehicle-model/)
- [ROS2 tf2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [geometry_msgs/PoseWithCovarianceStamped Message](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)