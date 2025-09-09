"""MCP Tools for Perception Media Bridge.

Provides tools for AI agents to access and analyze perception data from Autoware.
"""

from typing import Dict, Any, Optional

from ..perception_bridge import PerceptionBridge
from ..logging import get_logger

logger = get_logger(__name__)


class PerceptionTools:
    """Collection of perception-related MCP tools."""

    def __init__(self):
        """Initialize perception tools."""
        self.perception_bridge = PerceptionBridge()
        logger.info("Perception tools initialized")

    async def capture_camera_view(self, camera: str = "front") -> Dict[str, Any]:
        """
        Capture camera image for AI viewing.

        Args:
            camera: Which camera to capture from

        Returns:
            Dictionary with image_path that AI can read with Read tool
        """
        # Map camera names to topics
        camera_topics = {
            "front": "/sensing/camera/camera0/image_rect_color",
            "rear": "/sensing/camera/camera1/image_rect_color",
            "left": "/sensing/camera/camera2/image_rect_color",
            "right": "/sensing/camera/camera3/image_rect_color",
            "traffic_light": "/sensing/camera/traffic_light/image_raw",
        }

        topic = camera_topics.get(camera, camera_topics["front"])

        result = await self.perception_bridge.capture_camera_image(
            camera_topic=topic, timeout=5.0
        )

        if result.get("success"):
            return {
                "success": True,
                "image_path": result["image_path"],
                "message": f"Camera image saved to {result['image_path']}. Use Read tool to view.",
                "timestamp": result["timestamp"],
                "camera": camera,
                "dimensions": f"{result.get('width', 0)}x{result.get('height', 0)}",
            }
        else:
            return {
                "success": False,
                "error": result.get("error", "Failed to capture camera image"),
                "camera": camera,
            }

    async def visualize_lidar_scene(self, view_type: str = "bev") -> Dict[str, Any]:
        """
        Create LiDAR pointcloud visualization.

        Args:
            view_type: Type of visualization (bev, front, side, rear)

        Returns:
            Dictionary with visualization image path
        """
        result = await self.perception_bridge.visualize_pointcloud(
            pointcloud_topic="/sensing/lidar/concatenated/pointcloud",
            view_type=view_type,
            timeout=5.0,
        )

        if result.get("success"):
            return {
                "success": True,
                "image_path": result["image_path"],
                "message": f"LiDAR visualization saved to {result['image_path']}. Use Read tool to view.",
                "view_type": view_type,
                "num_points": result.get("num_points", 0),
                "timestamp": result["timestamp"],
            }
        else:
            return {
                "success": False,
                "error": result.get("error", "Failed to create LiDAR visualization"),
                "view_type": view_type,
            }

    async def get_perception_snapshot(self) -> Dict[str, Any]:
        """
        Get complete perception snapshot.

        Returns:
            Dictionary with multiple image paths and detection data
        """
        result = await self.perception_bridge.get_perception_snapshot()

        if result.get("success"):
            images = result.get("images", {})
            return {
                "success": True,
                "message": "Perception snapshot captured. Use Read tool to view images.",
                "camera_image": images.get("camera"),
                "lidar_visualization": images.get("lidar_bev"),
                "annotated_view": images.get("annotated"),
                "detected_objects": result.get("detected_objects", []),
                "num_objects": len(result.get("detected_objects", [])),
                "timestamp": result["timestamp"],
            }
        else:
            return {
                "success": False,
                "error": result.get("error", "Failed to get perception snapshot"),
            }

    async def analyze_driving_scene(self) -> Dict[str, Any]:
        """
        Comprehensive driving scene analysis.

        Returns:
            Complete scene analysis with images, objects, and recommendations
        """
        result = await self.perception_bridge.analyze_driving_scene()

        if result.get("success"):
            images = result.get("images", {})
            scene = result.get("scene_description", {})

            return {
                "success": True,
                "message": "Scene analysis complete. Use Read tool to view images.",
                "images": {
                    "camera": images.get("camera"),
                    "lidar_bev": images.get("lidar_bev"),
                    "annotated": images.get("annotated"),
                },
                "scene_summary": {
                    "num_vehicles": scene.get("num_vehicles", 0),
                    "num_pedestrians": scene.get("num_pedestrians", 0),
                    "traffic_light": scene.get("traffic_light_state", "unknown"),
                    "vehicle_speed_mps": scene.get("vehicle_speed_mps", 0),
                    "scene_type": scene.get("scene_type", "unknown"),
                },
                "detected_objects": result.get("detected_objects", []),
                "traffic_lights": result.get("traffic_lights", []),
                "recommendations": result.get("recommendations", []),
                "timestamp": result["timestamp"],
            }
        else:
            return {
                "success": False,
                "error": result.get("error", "Failed to analyze scene"),
            }

    async def get_detected_objects(self, object_type: str = "all") -> Dict[str, Any]:
        """
        Get list of detected objects.

        Args:
            object_type: Filter by type (all, vehicle, pedestrian, etc.)

        Returns:
            List of detected objects with properties
        """
        try:
            # Get all detected objects
            objects = await self.perception_bridge._get_detected_objects()

            # Filter by type if requested
            if object_type != "all":
                objects = [o for o in objects if o.get("type") == object_type]

            return {
                "success": True,
                "objects": objects,
                "count": len(objects),
                "object_type": object_type,
                "message": f"Found {len(objects)} objects of type '{object_type}'",
            }

        except Exception as e:
            logger.error(f"Error getting detected objects: {e}")
            return {"success": False, "error": str(e), "objects": [], "count": 0}

    def cleanup(self):
        """Clean up perception resources."""
        self.perception_bridge.cleanup()
        logger.info("Perception tools cleaned up")
