"""
ROS2 bridge for perception data.
"""

import asyncio
import numpy as np
from typing import Dict, Any, Optional, List, Callable
from concurrent.futures import ThreadPoolExecutor

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from std_msgs.msg import Header
    HAS_ROS2 = True
    try:
        from cv_bridge import CvBridge, CvBridgeError
        HAS_CV_BRIDGE = True
    except (ImportError, AttributeError):
        # cv_bridge might have numpy compatibility issues
        CvBridge = None
        CvBridgeError = Exception
        HAS_CV_BRIDGE = False
except ImportError:
    HAS_ROS2 = False
    HAS_CV_BRIDGE = False
    Node = object
    Image = None
    PointCloud2 = None
    CvBridge = None
    CvBridgeError = Exception


class PerceptionBridgeNode(Node if HAS_ROS2 else object):
    """ROS2 node for perception data bridging."""
    
    def __init__(self):
        """Initialize perception bridge node."""
        if not HAS_ROS2:
            raise RuntimeError("ROS2 is not available. Please source ROS2 environment.")
            
        super().__init__('perception_bridge')
        
        self.cv_bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.subscription_registry = {}
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.active_contexts = {}
        
        self.get_logger().info("Perception bridge node initialized")
        
    def add_image_subscription(self, topic: str, callback: Callable) -> None:
        """
        Add subscription to an image topic.
        
        Args:
            topic: ROS2 topic name
            callback: Callback function for received images
        """
        if topic in self.subscription_registry:
            return  # Already subscribed
            
        sub = self.create_subscription(
            Image,
            topic,
            lambda msg: self._handle_image(msg, topic, callback),
            10
        )
        self.subscription_registry[topic] = sub
        self.get_logger().info(f"Subscribed to image topic: {topic}")
        
    def add_pointcloud_subscription(self, topic: str, callback: Callable) -> None:
        """
        Add subscription to a point cloud topic.
        
        Args:
            topic: ROS2 topic name
            callback: Callback function for received point clouds
        """
        if topic in self.subscription_registry:
            return  # Already subscribed
            
        sub = self.create_subscription(
            PointCloud2,
            topic,
            lambda msg: self._handle_pointcloud(msg, topic, callback),
            10
        )
        self.subscription_registry[topic] = sub
        self.get_logger().info(f"Subscribed to pointcloud topic: {topic}")
        
    def remove_subscription(self, topic: str) -> None:
        """
        Remove subscription to a topic.
        
        Args:
            topic: Topic to unsubscribe from
        """
        if topic in self.subscription_registry:
            # ROS2 subscriptions are cleaned up automatically
            del self.subscription_registry[topic]
            self.get_logger().info(f"Unsubscribed from topic: {topic}")
            
    def _handle_image(self, msg: Image, topic: str, callback: Callable) -> None:
        """
        Handle incoming image message.
        
        Args:
            msg: ROS2 Image message
            topic: Topic name
            callback: Callback to process the image
        """
        try:
            if not self.cv_bridge:
                # If cv_bridge is not available, just pass raw data
                self.get_logger().warning("cv_bridge not available, passing raw image data")
                cv_image = None
            else:
                # Convert to OpenCV format (BGR8)
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Create capture data
            capture_data = {
                'type': 'image',
                'topic': topic,
                'data': cv_image,
                'timestamp_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                'frame_id': msg.header.frame_id,
                'encoding': msg.encoding,
                'dimensions': (msg.height, msg.width)
            }
            
            # Call the callback
            callback(capture_data)
            
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image from {topic}: {e}")
            
    def _handle_pointcloud(self, msg: PointCloud2, topic: str, callback: Callable) -> None:
        """
        Handle incoming point cloud message.
        
        Args:
            msg: ROS2 PointCloud2 message
            topic: Topic name
            callback: Callback to process the point cloud
        """
        try:
            # Convert to numpy array
            points = self._pointcloud2_to_numpy(msg)
            
            # Create capture data
            capture_data = {
                'type': 'pointcloud',
                'topic': topic,
                'data': points,
                'timestamp_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                'frame_id': msg.header.frame_id,
                'num_points': len(points),
                'height': msg.height,
                'width': msg.width
            }
            
            # Call the callback
            callback(capture_data)
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert pointcloud from {topic}: {e}")
            
    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """
        Convert PointCloud2 message to numpy array.
        
        Args:
            msg: PointCloud2 message
            
        Returns:
            Numpy array of shape (N, 4) with XYZI data
        """
        # Extract point data
        points_list = []
        
        # Parse fields to determine data structure
        field_names = [field.name for field in msg.fields]
        has_intensity = 'intensity' in field_names or 'i' in field_names
        
        # Simple parsing for common formats (XYZI)
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8)
        
        # Reshape based on message structure
        if msg.height == 1:
            # Unorganized point cloud
            num_points = msg.width
        else:
            # Organized point cloud
            num_points = msg.height * msg.width
            
        # Extract XYZ (and I if available)
        points = np.zeros((num_points, 4), dtype=np.float32)
        
        for i in range(num_points):
            base_idx = i * point_step
            # Extract X, Y, Z (assuming float32)
            points[i, 0] = np.frombuffer(data[base_idx:base_idx+4], dtype=np.float32)[0]  # X
            points[i, 1] = np.frombuffer(data[base_idx+4:base_idx+8], dtype=np.float32)[0]  # Y
            points[i, 2] = np.frombuffer(data[base_idx+8:base_idx+12], dtype=np.float32)[0]  # Z
            
            if has_intensity and point_step >= 16:
                # Extract intensity if available
                points[i, 3] = np.frombuffer(data[base_idx+12:base_idx+16], dtype=np.float32)[0]
            else:
                points[i, 3] = 0.0  # Default intensity
                
        # Filter out invalid points (NaN or Inf)
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        
        return points
        
    def get_topic_type(self, topic: str) -> Optional[str]:
        """
        Get the message type of a topic.
        
        Args:
            topic: Topic name
            
        Returns:
            Message type string or None
        """
        topic_names_and_types = self.get_topic_names_and_types()
        for name, types in topic_names_and_types:
            if name == topic and types:
                return types[0]
        return None


# Singleton instance
_bridge_node = None
_ros2_initialized = False


def get_bridge_node() -> Optional[PerceptionBridgeNode]:
    """
    Get or create the singleton perception bridge node.
    
    Returns:
        PerceptionBridgeNode instance or None if ROS2 not available
    """
    global _bridge_node, _ros2_initialized
    
    if not HAS_ROS2:
        return None
        
    if not _ros2_initialized:
        try:
            rclpy.init()
            _ros2_initialized = True
        except Exception:
            # Already initialized
            _ros2_initialized = True
            
    if _bridge_node is None:
        _bridge_node = PerceptionBridgeNode()
        
    return _bridge_node