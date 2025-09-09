# Perception Bridge Implementation Guide

This guide provides detailed implementation instructions for integrating CV_Bridge and Open3D into the Autoware MCP perception bridge.

## Prerequisites

### Required Dependencies

Add the following dependencies to `pyproject.toml`:

```toml
dependencies = [
    # Existing dependencies...
    "numpy>=1.24.0",
    "opencv-python>=4.8.0",
    "open3d>=0.17.0",
    "pillow>=10.0.0",  # For image manipulation
]
```

### ROS2 Dependencies

Install ROS2 Python packages for image and point cloud handling:

```bash
# Install cv_bridge for ROS2 Humble
sudo apt-get install ros-humble-cv-bridge python3-cv-bridge

# Install point cloud processing tools
sudo apt-get install ros-humble-sensor-msgs-py
```

## CV_Bridge Implementation

### Step 1: Create ROS2 Image Subscriber

```python
# src/autoware_mcp/perception/image_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pathlib import Path
from datetime import datetime
import asyncio
from typing import Dict, Any, Optional

class ImageSubscriberNode(Node):
    """ROS2 node for subscribing to camera images."""
    
    def __init__(self):
        super().__init__('perception_image_subscriber')
        
        self.bridge = CvBridge()
        self.media_dir = Path('/tmp/autoware_mcp/perception')
        self.media_dir.mkdir(parents=True, exist_ok=True)
        
        # Camera topic mapping
        self.camera_topics = {
            'front': '/sensing/camera/camera0/image_rect_color',
            'rear': '/sensing/camera/camera1/image_rect_color',
            'left': '/sensing/camera/camera2/image_rect_color',
            'right': '/sensing/camera/camera3/image_rect_color',
            'traffic_light': '/sensing/camera/traffic_light/image_raw'
        }
        
        # Latest images cache
        self.latest_images = {}
        
        # Create subscribers for all cameras
        self.subscribers = {}
        for camera_name, topic in self.camera_topics.items():
            self.subscribers[camera_name] = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=camera_name: self.image_callback(msg, cam),
                1  # QoS depth
            )
            self.get_logger().info(f'Subscribed to {topic} for {camera_name} camera')
    
    def image_callback(self, msg: Image, camera_name: str):
        """Handle incoming image messages."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Save to file
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'camera_{camera_name}_{timestamp}.jpg'
            filepath = self.media_dir / filename
            
            # Apply JPEG compression
            cv2.imwrite(str(filepath), cv_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            
            # Cache latest image path
            self.latest_images[camera_name] = {
                'path': str(filepath),
                'timestamp': datetime.now().isoformat(),
                'width': cv_image.shape[1],
                'height': cv_image.shape[0]
            }
            
            self.get_logger().debug(f'Saved {camera_name} image to {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to process {camera_name} image: {e}')
    
    async def capture_camera_image(self, camera_name: str = 'front') -> Dict[str, Any]:
        """Capture and return the latest image from specified camera."""
        if camera_name in self.latest_images:
            return {
                'success': True,
                **self.latest_images[camera_name]
            }
        else:
            return {
                'success': False,
                'error': f'No image available from {camera_name} camera'
            }
```

### Step 2: Integrate with Perception Bridge

```python
# Update src/autoware_mcp/perception_bridge.py

import rclpy
from rclpy.executors import MultiThreadedExecutor
from .perception.image_subscriber import ImageSubscriberNode

class PerceptionBridge:
    """Enhanced perception bridge with ROS2 integration."""
    
    def __init__(self):
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create and spin image subscriber node in background
        self.image_node = ImageSubscriberNode()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.image_node)
        
        # Start executor in background thread
        import threading
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )
        self.spin_thread.start()
        
        # Rest of initialization...
    
    async def capture_camera_image(
        self,
        camera_topic: str = "front",
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """Capture camera image using ROS2 subscriber."""
        return await self.image_node.capture_camera_image(camera_topic)
    
    def cleanup(self):
        """Clean up ROS2 resources."""
        if hasattr(self, 'executor'):
            self.executor.shutdown()
        if hasattr(self, 'image_node'):
            self.image_node.destroy_node()
        # Clean up media files...
```

## Open3D Point Cloud Rendering

### Step 1: Create Point Cloud Processor

```python
# src/autoware_mcp/perception/pointcloud_processor.py

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from typing import Dict, Any, Tuple
import cv2
from pathlib import Path

class PointCloudProcessor:
    """Process and render point cloud data."""
    
    def __init__(self):
        self.media_dir = Path('/tmp/autoware_mcp/perception')
        self.media_dir.mkdir(parents=True, exist_ok=True)
    
    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array."""
        points_list = []
        
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        
        return np.array(points_list)
    
    def create_bev_image(
        self,
        points: np.ndarray,
        resolution: float = 0.1,
        x_range: Tuple[float, float] = (-50, 50),
        y_range: Tuple[float, float] = (-50, 50),
        z_range: Tuple[float, float] = (-2, 5)
    ) -> np.ndarray:
        """Create bird's eye view image from points."""
        
        # Filter points by range
        mask = (
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )
        points_filtered = points[mask]
        
        # Calculate image size
        width = int((x_range[1] - x_range[0]) / resolution)
        height = int((y_range[1] - y_range[0]) / resolution)
        
        # Create empty image
        bev_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Project points to image
        for point in points_filtered:
            x, y, z = point
            
            # Convert to pixel coordinates
            px = int((x - x_range[0]) / resolution)
            py = int((y - y_range[0]) / resolution)
            
            # Ensure within bounds
            if 0 <= px < width and 0 <= py < height:
                # Color based on height
                intensity = self._height_to_intensity(z, z_range)
                color = self._intensity_to_color(intensity)
                
                # Set pixel (flip y-axis for image coordinates)
                bev_image[height - py - 1, px] = color
        
        # Apply morphological operations for better visualization
        kernel = np.ones((2, 2), np.uint8)
        bev_image = cv2.dilate(bev_image, kernel, iterations=1)
        
        return bev_image
    
    def _height_to_intensity(
        self,
        z: float,
        z_range: Tuple[float, float]
    ) -> float:
        """Convert height to normalized intensity."""
        return np.clip((z - z_range[0]) / (z_range[1] - z_range[0]), 0, 1)
    
    def _intensity_to_color(self, intensity: float) -> np.ndarray:
        """Convert intensity to RGB color (jet colormap)."""
        # Simple jet colormap implementation
        if intensity < 0.25:
            r = 0
            g = int(4 * intensity * 255)
            b = 255
        elif intensity < 0.5:
            r = 0
            g = 255
            b = int((2 - 4 * intensity) * 255)
        elif intensity < 0.75:
            r = int((4 * intensity - 2) * 255)
            g = 255
            b = 0
        else:
            r = 255
            g = int((4 - 4 * intensity) * 255)
            b = 0
        
        return np.array([b, g, r], dtype=np.uint8)  # BGR for OpenCV
    
    def render_3d_view(
        self,
        points: np.ndarray,
        view_params: Dict[str, Any] = None
    ) -> np.ndarray:
        """Render 3D view using Open3D."""
        
        # Default view parameters
        if view_params is None:
            view_params = {
                'camera_position': [0, -30, 20],
                'look_at': [0, 0, 0],
                'up_vector': [0, 0, 1],
                'fov': 60,
                'width': 1920,
                'height': 1080
            }
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Add colors based on height
        z_values = points[:, 2]
        colors = self._generate_height_colors(z_values)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Create visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            visible=False,
            width=view_params['width'],
            height=view_params['height']
        )
        
        # Add point cloud
        vis.add_geometry(pcd)
        
        # Set view parameters
        view_control = vis.get_view_control()
        view_control.set_lookat(view_params['look_at'])
        view_control.set_up(view_params['up_vector'])
        view_control.set_front(view_params['camera_position'])
        view_control.set_zoom(0.5)
        
        # Render to image
        vis.poll_events()
        vis.update_renderer()
        image = vis.capture_screen_float_buffer(do_render=True)
        
        # Convert to numpy array
        image_np = np.asarray(image) * 255
        image_np = image_np.astype(np.uint8)
        
        # Clean up
        vis.destroy_window()
        
        return image_np
    
    def _generate_height_colors(self, z_values: np.ndarray) -> np.ndarray:
        """Generate colors based on height values."""
        z_min, z_max = z_values.min(), z_values.max()
        z_normalized = (z_values - z_min) / (z_max - z_min + 1e-6)
        
        colors = np.zeros((len(z_values), 3))
        for i, intensity in enumerate(z_normalized):
            color = self._intensity_to_color(intensity)
            colors[i] = color / 255.0  # Open3D expects [0, 1] range
        
        return colors
```

### Step 2: Create Point Cloud Subscriber

```python
# src/autoware_mcp/perception/pointcloud_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from datetime import datetime
import cv2
from .pointcloud_processor import PointCloudProcessor

class PointCloudSubscriberNode(Node):
    """ROS2 node for subscribing to point cloud data."""
    
    def __init__(self):
        super().__init__('perception_pointcloud_subscriber')
        
        self.processor = PointCloudProcessor()
        self.latest_pointcloud = None
        
        # Subscribe to concatenated point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            self.pointcloud_callback,
            1
        )
        
        self.get_logger().info('Subscribed to point cloud topic')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Handle incoming point cloud messages."""
        self.latest_pointcloud = msg
        self.get_logger().debug(f'Received pointcloud with {msg.width} points')
    
    async def render_pointcloud(
        self,
        view_type: str = 'bev'
    ) -> Dict[str, Any]:
        """Render the latest point cloud."""
        
        if self.latest_pointcloud is None:
            return {
                'success': False,
                'error': 'No point cloud data available'
            }
        
        try:
            # Convert to numpy array
            points = self.processor.pointcloud2_to_array(self.latest_pointcloud)
            
            # Render based on view type
            if view_type == 'bev':
                image = self.processor.create_bev_image(points)
            elif view_type == '3d':
                image = self.processor.render_3d_view(points)
            else:
                return {
                    'success': False,
                    'error': f'Unknown view type: {view_type}'
                }
            
            # Save image
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'pointcloud_{view_type}_{timestamp}.png'
            filepath = self.processor.media_dir / filename
            
            cv2.imwrite(str(filepath), image)
            
            return {
                'success': True,
                'image_path': str(filepath),
                'num_points': len(points),
                'view_type': view_type,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to render point cloud: {e}')
            return {
                'success': False,
                'error': str(e)
            }
```

## Testing the Implementation

### Unit Tests

```python
# tests/test_perception_implementation.py

import pytest
import numpy as np
from unittest.mock import Mock, patch
from autoware_mcp.perception.pointcloud_processor import PointCloudProcessor
from sensor_msgs.msg import PointCloud2

def test_bev_rendering():
    """Test bird's eye view rendering."""
    processor = PointCloudProcessor()
    
    # Create sample points
    points = np.random.randn(1000, 3) * 10
    
    # Render BEV
    image = processor.create_bev_image(points)
    
    assert image.shape == (1000, 1000, 3)  # Default 100m x 100m at 0.1m resolution
    assert image.dtype == np.uint8

def test_height_coloring():
    """Test height-based coloring."""
    processor = PointCloudProcessor()
    
    # Test different heights
    intensity_low = processor._height_to_intensity(-2, (-2, 5))
    intensity_mid = processor._height_to_intensity(1.5, (-2, 5))
    intensity_high = processor._height_to_intensity(5, (-2, 5))
    
    assert intensity_low == 0.0
    assert 0.4 < intensity_mid < 0.6
    assert intensity_high == 1.0

@patch('cv2.imwrite')
def test_image_saving(mock_imwrite):
    """Test that images are saved correctly."""
    processor = PointCloudProcessor()
    points = np.random.randn(100, 3)
    
    image = processor.create_bev_image(points)
    
    # Verify image can be saved
    filepath = processor.media_dir / 'test.png'
    cv2.imwrite(str(filepath), image)
    
    mock_imwrite.assert_called_once()
```

### Integration Test

```python
# tests/test_perception_integration.py

import pytest
import asyncio
from autoware_mcp.perception_bridge import PerceptionBridge

@pytest.mark.asyncio
async def test_full_perception_pipeline():
    """Test complete perception pipeline."""
    bridge = PerceptionBridge()
    
    try:
        # Test camera capture
        camera_result = await bridge.capture_camera_image('front')
        assert 'image_path' in camera_result or 'error' in camera_result
        
        # Test point cloud visualization
        lidar_result = await bridge.visualize_pointcloud('bev')
        assert 'image_path' in lidar_result or 'error' in lidar_result
        
        # Test perception snapshot
        snapshot = await bridge.get_perception_snapshot()
        assert 'images' in snapshot or 'error' in snapshot
        
    finally:
        bridge.cleanup()
```

## Performance Optimizations

### 1. Image Compression

```python
# Optimize JPEG quality based on use case
quality_settings = {
    'preview': 70,    # Fast, smaller files
    'analysis': 90,   # High quality for AI
    'archive': 95     # Maximum quality
}
```

### 2. Point Cloud Decimation

```python
# Reduce points for faster processing
def decimate_pointcloud(points: np.ndarray, factor: int = 10) -> np.ndarray:
    """Decimate point cloud by keeping every Nth point."""
    return points[::factor]
```

### 3. Concurrent Processing

```python
# Process multiple views in parallel
import concurrent.futures

async def render_all_views(points: np.ndarray) -> Dict[str, str]:
    """Render all views concurrently."""
    view_types = ['bev', 'front', 'side', 'iso']
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        futures = {
            view: executor.submit(render_view, points, view)
            for view in view_types
        }
        
        results = {}
        for view, future in futures.items():
            results[view] = await asyncio.wrap_future(future)
        
        return results
```

## Deployment Checklist

Before deploying the enhanced perception bridge:

1. **Install Dependencies**
   - [ ] opencv-python installed
   - [ ] open3d installed
   - [ ] cv_bridge ROS2 package installed
   - [ ] sensor_msgs_py installed

2. **Verify ROS2 Topics**
   - [ ] Camera topics publishing
   - [ ] Point cloud topics publishing
   - [ ] Message types match expected formats

3. **Configure Paths**
   - [ ] Media directory writable
   - [ ] Cleanup policy configured
   - [ ] File permissions set correctly

4. **Test Components**
   - [ ] Image capture working
   - [ ] Point cloud rendering working
   - [ ] File cleanup working
   - [ ] Memory usage acceptable

5. **Performance Validation**
   - [ ] Processing latency < 100ms
   - [ ] Memory usage < 2GB
   - [ ] Disk usage managed

## Troubleshooting

### Common Issues

1. **CV_Bridge Import Error**
   ```bash
   # Fix: Source ROS2 environment
   source /opt/ros/humble/setup.bash
   ```

2. **Open3D Headless Rendering**
   ```bash
   # Fix: Set environment variable
   export DISPLAY=:0  # or use xvfb-run
   ```

3. **Permission Denied on Media Directory**
   ```bash
   # Fix: Set proper permissions
   chmod 755 /tmp/autoware_mcp/perception
   ```

## Next Steps

1. Implement real-time streaming with WebRTC
2. Add semantic segmentation overlays
3. Integrate with traffic light recognition
4. Add 360° panoramic stitching
5. Implement temporal filtering for noise reduction