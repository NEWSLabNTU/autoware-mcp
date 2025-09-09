# Perception Tools API Reference

## Overview

The Perception Bridge provides 6 clean MCP tools for accessing and analyzing Autoware's sensor data. All tools work with a unified context system that manages buffers, storage, and synchronization.

## Tool Reference

### 1. create_perception_context

Create a new perception context for sensor operations.

**Parameters:**
- `context_type` (str, default="general"): Context type for optimization
  - Options: "general", "highway", "parking", "emergency", "urban", "rural"
- `buffer_duration` (float, default=30.0): Rolling buffer size in seconds
- `subscriptions` (List[str], optional): Topics to pre-subscribe

**Returns:**
- `PerceptionContext`: Context object for use with other tools

**Example:**
```python
context = await create_perception_context(
    context_type="highway",
    buffer_duration=60.0,
    subscriptions=[
        "/sensing/camera/camera0/image_rect_color",
        "/sensing/lidar/concatenated/pointcloud"
    ]
)
```

### 2. capture_sensors

Capture data from multiple sensors with synchronization.

**Parameters:**
- `topics` (List[str]): Any sensor topics to capture from
- `context` (PerceptionContext): Context to use for capture
- `time_sync` (bool, default=True): Whether to synchronize captures
- `return_individual` (bool, default=True): Return individual files for AI to read

**Returns:**
- `CaptureResult`: Capture result with individual sensor data files

**Example:**
```python
capture = await capture_sensors(
    topics=[
        "/sensing/camera/camera0/image_rect_color",
        "/sensing/camera/camera1/image_rect_color",
        "/sensing/lidar/concatenated/pointcloud"
    ],
    context=context,
    time_sync=True,
    return_individual=True
)

# Each camera image saved separately
# capture.captures["/sensing/camera/camera0/image_rect_color"].file_path
# AI can read each image with Read tool
```

### 3. generate_visualization

Generate visualization with specified projection and overlays.

**Parameters:**
- `source` (Union[str, CaptureResult]): Topic name or previous capture
- `projection` (str): Projection type (only for point clouds, ignored for images)
- `projection_params` (Optional[ProjectionParams]): Custom projection parameters
- `overlays` (List[OverlayConfig], optional): Overlays to apply
- `scale` (float, default=0.1): Scale in meters per pixel (for point clouds)
- `focus` (Optional[FocusConfig]): AI-controlled focus area
- `context` (PerceptionContext): Context to use
- `point_color_mode` (str, default="height"): Coloring mode for point clouds
- `color_params` (Dict[str, Any], optional): Mode-specific color parameters

**Projection Types:**
- `"bev"`: Bird's eye view
- `"front"`, `"rear"`, `"left"`, `"right"`: Standard orthographic views
- `"angled"`: Custom angle with azimuth/elevation parameters
- `"perspective"`: Perspective view with FOV settings
- `"cylindrical"`: 360° unwrapped view
- `"spherical"`: Equirectangular projection
- `"range_image"`: LiDAR native format
- **Presets**: `"front_left_45"`, `"rear_right_135"`, etc.

**Point Cloud Color Modes:**
- `"height"`: Color by Z coordinate (elevation)
- `"distance"`: Color by 3D distance from sensor
- `"intensity"`: Color by LiDAR return intensity
- `"uniform"`: Single color for all points
- `"radial_distance"`: Color by XY plane distance only
- `"azimuth"`: Color by horizontal angle
- `"ring"`: Color by LiDAR ring/layer (if available)

**Returns:**
- `Visualization`: Visualization with image path and metadata

**Note:** 
- For camera images: Only overlays are applied, no projection
- For point clouds: Projection is applied with coloring, then overlays

**Example:**
```python
# Standard BEV with distance coloring
lidar_viz = await generate_visualization(
    source="/sensing/lidar/concatenated/pointcloud",
    projection="bev",
    point_color_mode="distance",
    color_params={
        "max_distance": 100.0,
        "colormap": "plasma",
        "use_log_scale": True
    },
    scale=0.05,
    context=context
)

# 45-degree front-left view using preset
intersection_view = await generate_visualization(
    source="/sensing/lidar/concatenated/pointcloud",
    projection="front_left_45",  # Preset angle
    point_color_mode="distance",
    context=context
)

# Custom angle with parameters
custom_view = await generate_visualization(
    source="/sensing/lidar/concatenated/pointcloud",
    projection="angled",
    projection_params=ProjectionParams(
        azimuth=110,  # Rear-left angle
        elevation=20,  # Slightly elevated
        x_range=(-30, 30),
        y_range=(-20, 20),
        ground_removal=True,
        clip_distance=50
    ),
    point_color_mode="radial_distance",
    context=context
)

# Driver perspective view
driver_view = await generate_visualization(
    source="/sensing/lidar/concatenated/pointcloud",
    projection="perspective",
    projection_params=ProjectionParams(
        fov_horizontal=120,
        fov_vertical=80,
        camera_height=1.2,
        look_ahead=15
    ),
    point_color_mode="distance",
    color_params={"use_log_scale": True},
    context=context
)
```

### 4. analyze_temporal

Analyze temporal patterns in perception data.

**Parameters:**
- `analysis_type` (str): Type of analysis
  - Options: "motion", "change", "emergence"
- `context` (PerceptionContext): Context to analyze
- `time_window` (Optional[Tuple[float, float]]): Time range to analyze
- `visualization` (bool, default=True): Generate visualization

**Returns:**
- `TemporalAnalysis`: Analysis results with optional visualization

**Example:**
```python
motion = await analyze_temporal(
    analysis_type="motion",
    context=context,
    time_window=(-5.0, 0.0),  # Last 5 seconds
    visualization=True
)
```

### 5. get_multi_sensor_view

Capture multiple sensors and return organized individual images.

**Parameters:**
- `sensor_groups` (Dict[str, List[str]]): Sensors grouped by purpose
- `context` (PerceptionContext): Context to use
- `layout` (str, default="grid"): How to organize results
  - Options: "grid", "list", "grouped"

**Returns:**
- `MultiSensorResult`: Organized collection of individual sensor captures

**Note:** No stitching or transformation - each image is saved separately for AI to read individually.

**Example:**
```python
multi_view = await get_multi_sensor_view(
    sensor_groups={
        "forward": ["/sensing/camera/camera0/image_rect_color"],
        "rear": ["/sensing/camera/camera1/image_rect_color"],
        "sides": [
            "/sensing/camera/camera2/image_rect_color",  # Left
            "/sensing/camera/camera3/image_rect_color"   # Right
        ],
        "lidar": ["/sensing/lidar/concatenated/pointcloud"]
    },
    context=context,
    layout="grouped"
)

# Access individual images
front_image_path = multi_view.sensor_groups["forward"][0].file_path
left_side_path = multi_view.sensor_groups["sides"][0].file_path

# AI reads each image separately with Read tool
# No assumptions about camera calibration or model
```

### 6. save_context_buffer

Save context buffer for later analysis.

**Parameters:**
- `context` (PerceptionContext): Context to save
- `duration` (Optional[float]): Save last N seconds (None = all)
- `trigger` (Optional[str]): Event trigger name

**Returns:**
- `str`: Archive ID for retrieval

**Example:**
```python
archive_id = await save_context_buffer(
    context=context,
    duration=10.0,  # Last 10 seconds
    trigger="emergency_brake"
)
```

## Data Types

### PerceptionContext

Manages the lifecycle of perception data capture and analysis.

**Attributes:**
- `context_id` (str): Unique context identifier
- `rolling_buffer` (CircularBuffer): Rolling data buffer
- `context_type` (str): Context optimization type
- `storage_path` (Path): Storage location

### CaptureResult

Unified capture result from sensors.

**Attributes:**
- `context_id` (str): Associated context ID
- `timestamp_ns` (int): Capture timestamp in nanoseconds
- `captures` (Dict[str, Union[ImageData, PointCloudData]]): Captured data by topic
- `synchronization_offset_ns` (Optional[int]): Sync offset if applicable

### Visualization

Generated visualization result.

**Attributes:**
- `image_path` (str): Path to saved visualization
- `metadata` (Dict): Visualization metadata
- `projection` (str): Applied projection type
- `overlays` (List[str]): Applied overlays

### TemporalAnalysis

Results from temporal analysis.

**Attributes:**
- `analysis_type` (str): Type of analysis performed
- `time_window` (Tuple[float, float]): Analyzed time range
- `results` (Dict): Analysis-specific results
- `visualization_path` (Optional[str]): Path to visualization if generated

### MultiSensorResult

Result from multi-sensor capture without stitching.

**Attributes:**
- `context_id` (str): Associated context ID
- `timestamp_ns` (int): Capture timestamp
- `sensor_groups` (Dict[str, List[SensorCapture]]): Sensors grouped by purpose
- `layout` (str): Organization layout ("grid", "list", "grouped")
- `metadata` (Dict): Additional metadata

**Methods:**
- `get_all_image_paths()`: Get all image paths for AI to read
- `get_group_images(group)`: Get images for specific group

### SensorCapture

Individual sensor capture within MultiSensorResult.

**Attributes:**
- `topic` (str): ROS topic name
- `type` (str): "image" or "pointcloud"
- `file_path` (str): Path to saved file for AI to read
- `metadata` (Dict): Frame info, encoding, etc.

## Projection Types (Point Clouds Only)

Projections are only applied to 3D point cloud data. Camera images are used as-is without transformation.

### Standard Views
- `bev`: Bird's eye view (most common for driving)
- `front`: Front-facing view (0°)
- `rear`: Rear-facing view (180°)
- `left`: Left side view (90°)
- `right`: Right side view (-90°)

### Custom Angles
- `angled`: Specify any azimuth/elevation with parameters
- `perspective`: Perspective projection with FOV

### Projection Presets
Common angles available as presets:
- `front_left_30`, `front_left_45`, `front_left_60`
- `front_right_30`, `front_right_45`, `front_right_60`
- `rear_left_120`, `rear_left_135`
- `rear_right_120`, `rear_right_135`
- `elevated_front`: Front view from above
- `elevated_bev`: Nearly top-down view
- `driver_view`: Driver's eye perspective

### Advanced Projections
- `cylindrical`: 360° unwrapped cylindrical projection
- `spherical`: Equirectangular spherical projection
- `range_image`: LiDAR native range image format

### ProjectionParams

Parameters for customizing projections:

```python
@dataclass
class ProjectionParams:
    # Angle parameters (for "angled" projection)
    azimuth: float = 0.0          # Horizontal angle (-180 to 180°)
    elevation: float = 0.0        # Vertical angle (-90 to 90°)
    
    # Perspective parameters
    fov_horizontal: float = 90.0  # Horizontal field of view
    fov_vertical: float = 60.0    # Vertical field of view  
    camera_height: float = 1.5    # Camera height (meters)
    look_ahead: float = 10.0      # Look-ahead distance (meters)
    
    # View range parameters
    x_range: Tuple[float, float] = (-50, 50)  # X range (meters)
    y_range: Tuple[float, float] = (-30, 30)  # Y range (meters)
    z_range: Tuple[float, float] = (-2, 10)   # Z filter range
    
    # Common parameters
    clip_distance: float = 100.0   # Max render distance
    ground_removal: bool = True    # Remove ground plane
    ground_threshold: float = -1.5 # Ground height threshold
```

**Example Usage:**
```python
# Custom 110° rear-left view
params = ProjectionParams(
    azimuth=110,
    elevation=15,
    x_range=(-25, 25),
    ground_removal=True
)

viz = await generate_visualization(
    source="/sensing/lidar/concatenated/pointcloud",
    projection="angled",
    projection_params=params,
    context=context
)
```

**Important Notes:**
- Camera images are NOT projected or transformed
- Each camera's intrinsics/distortion are preserved
- AI reads original camera images directly
- Only point clouds need projection to 2D

## Overlay Types

Available overlays for enhancing visualizations:

### Motion Overlays
- `velocity_vectors`: Object velocity arrows
- `trajectory_trails`: Historical movement paths
- `predicted_paths`: Predicted future trajectories

### Measurement Overlays
- `distance_grid`: Distance measurement grid
- `clearance_zones`: Safety clearance zones
- `proximity_warning`: Proximity alerts

### Detection Overlays
- `object_boxes`: Object bounding boxes
- `confidence_heatmap`: Detection confidence visualization
- `occlusion_mask`: Occluded area highlighting

### Analysis Overlays
- `change_highlight`: Highlight changed areas
- `emergence_marker`: Mark newly appeared objects
- `attention_focus`: AI attention areas

## Configuration

### Point Cloud Color Parameters

Parameters for different point cloud coloring modes.

#### Height Coloring
```python
{
    "min_height": -2.0,      # Minimum height for mapping (meters)
    "max_height": 5.0,       # Maximum height for mapping (meters)
    "colormap": "viridis"    # Matplotlib colormap name
}
```

#### Distance Coloring
```python
{
    "max_distance": 100.0,   # Maximum distance for normalization (meters)
    "colormap": "plasma",    # Matplotlib colormap name
    "use_log_scale": False   # Use logarithmic scale for better contrast
}
```

#### Intensity Coloring
```python
{
    "min_intensity": 0.0,    # Minimum intensity value
    "max_intensity": 255.0,  # Maximum intensity value
    "enhance_contrast": True # Apply histogram equalization
}
```

#### Uniform Coloring
```python
{
    "color": (0, 255, 0)     # BGR color tuple
}
```

#### Radial Distance Coloring
```python
{
    "max_radius": 50.0,      # Maximum XY distance (meters)
    "colormap": "jet",       # Matplotlib colormap name
    "highlight_close": True  # Emphasize nearby objects
}
```

**Available Colormaps:**
- `viridis`: Green to purple (good for height)
- `plasma`: Purple to yellow (good for distance)
- `jet`: Blue to red rainbow
- `hot`: Black to red to white (good for proximity)
- `gray`: Grayscale
- `terrain`: Terrain-like colors
- `hsv`: Full hue spectrum (good for angles)

### OverlayConfig

Configuration for visualization overlays.

**Fields:**
- `type` (str): Overlay type from above list
- `params` (Dict[str, Any]): Type-specific parameters
- `opacity` (float, default=0.7): Overlay transparency (0-1)

**Example:**
```python
overlay = OverlayConfig(
    type="distance_grid",
    params={
        "resolution": 0.5,  # Grid resolution in meters
        "max_distance": 50.0,  # Maximum distance to show
        "color": [0, 255, 0]  # Grid color (BGR)
    },
    opacity=0.5
)
```

### FocusConfig

Configuration for AI-controlled focus area.

**Fields:**
- `center` (Tuple[float, float]): Focus center (x, y) in meters
- `radius` (float): Focus radius in meters
- `blur_outside` (bool): Whether to blur outside focus area
- `highlight` (bool): Whether to highlight focus area

**Example:**
```python
focus = FocusConfig(
    center=(10.0, 0.0),  # 10m ahead
    radius=5.0,
    blur_outside=True,
    highlight=True
)
```

## Error Handling

All perception tools follow consistent error handling:

### Common Errors

**TopicNotAvailable**
- Topic does not exist or has no publishers
- Solution: Check topic name and ensure sensor is running

**SynchronizationTimeout**
- Could not synchronize captures within time limit
- Solution: Increase `max_time_diff` parameter

**BufferOverflow**
- Context buffer exceeded memory limit
- Solution: Reduce `buffer_duration` or save and clear buffer

**ProjectionError**
- Invalid projection type or insufficient data
- Solution: Check projection name and ensure data type matches

**StorageError**
- Could not save to disk
- Solution: Check disk space and permissions

### Error Response Format

```python
{
    "success": False,
    "error": "TopicNotAvailable",
    "message": "Topic '/sensing/camera/camera99/image' not found",
    "details": {
        "available_topics": [...],
        "suggestion": "Did you mean '/sensing/camera/camera0/image_rect_color'?"
    }
}
```

## Performance Considerations

### Buffer Management
- Default 30-second buffer uses ~1GB RAM
- Adjust `buffer_duration` based on available memory
- Use `save_context_buffer` to archive old data

### Parallel Processing
- Captures from multiple topics happen in parallel
- Visualization generation uses process pool
- Cache frequently used visualizations

### Storage Optimization
- Images saved as JPEG with quality=95
- Point clouds compressed when archived
- Automatic cleanup of old contexts

### Real-time Constraints
- Image capture: ~5ms latency
- Point cloud capture: ~15ms latency
- BEV generation: ~30ms for 100k points
- Keep total pipeline under 100ms for real-time

## Usage Patterns

### Pattern 1: Continuous Monitoring
```python
# Create long-running context
context = await create_perception_context(
    context_type="highway",
    buffer_duration=120.0  # 2 minutes
)

# Continuous capture loop
while driving:
    capture = await capture_sensors(
        topics=all_sensor_topics,
        context=context
    )
    
    # AI processes capture
    decision = ai_model.process(capture)
    
    # Save on interesting events
    if decision.is_interesting:
        await save_context_buffer(
            context=context,
            duration=30.0,
            trigger=decision.event_type
        )
```

### Pattern 2: Event-Triggered Analysis
```python
# On emergency brake
context = get_active_context()

# Save recent history
archive = await save_context_buffer(
    context=context,
    duration=10.0,
    trigger="emergency"
)

# Analyze what happened
analysis = await analyze_temporal(
    analysis_type="change",
    context=context,
    time_window=(-5.0, 0.0)
)

# Generate comprehensive view
view = await generate_visualization(
    source=context.get_latest_capture(),
    projection="multi_6",
    overlays=[
        OverlayConfig(type="change_highlight"),
        OverlayConfig(type="velocity_vectors")
    ],
    context=context
)
```

### Pattern 3: Parking Assistance
```python
# Create parking context
context = await create_perception_context(
    context_type="parking",
    buffer_duration=10.0
)

# Generate overhead view
overhead = await create_synthetic_view(
    view_type="parking_assist",
    camera_topics=all_cameras,
    context=context,
    overlays=["distance_grid", "clearance_zones"]
)

# Get precise measurements
clearance = await generate_visualization(
    source="/sensing/lidar/concatenated/pointcloud",
    projection="bev",
    overlays=[
        OverlayConfig(
            type="clearance_zones",
            params={
                "critical": 0.2,
                "warning": 0.5,
                "safe": 1.0
            }
        )
    ],
    scale=0.02,  # 2cm per pixel
    context=context
)
```

## Migration from v1.0

If migrating from the original design:

1. Replace multiple capture tools with single `capture_sensors`
2. Use contexts instead of recording IDs
3. Projections are now parameters, not separate tools
4. Overlays are composable configurations
5. Storage is automatic within contexts

## See Also

- [Perception Bridge Architecture](../../architecture/perception-bridge-v2.md)
- [Perception Implementation Guide](../perception-implementation.md)
- [MCP Tools Overview](./mcp-tools.md)