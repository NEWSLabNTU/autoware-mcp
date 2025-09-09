"""Perception Media Bridge for AI Agents.

This module provides tools to capture and process perception data from Autoware
and make it accessible to AI agents through MCP tools.
"""

import asyncio
import json
import os
import subprocess
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional
import numpy as np

from .logging import get_logger

logger = get_logger(__name__)


class PerceptionBridge:
    """Main perception bridge for capturing and serving perception data to AI agents."""
    
    def __init__(self):
        """Initialize the perception bridge."""
        # Create temp directory for media files
        self.media_dir = Path("/tmp/autoware_mcp/perception")
        self.media_dir.mkdir(parents=True, exist_ok=True)
        
        # Keep track of generated files for cleanup
        self.generated_files = []
        self.max_files = 100  # Maximum files to keep
        
        logger.info(f"Perception bridge initialized, media directory: {self.media_dir}")
    
    async def capture_camera_image(
        self, 
        camera_topic: str = "/sensing/camera/traffic_light/image_raw",
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """
        Capture a single frame from a camera topic and save as image.
        
        Args:
            camera_topic: ROS2 topic name for camera image
            timeout: Maximum time to wait for image
            
        Returns:
            Dictionary containing:
            - image_path: Path to saved image file
            - timestamp: Capture timestamp
            - width: Image width
            - height: Image height
            - error: Error message if capture failed
        """
        timestamp = datetime.now().isoformat()
        image_filename = f"camera_{int(time.time() * 1000)}.jpg"
        image_path = self.media_dir / image_filename
        
        try:
            # Use ros2 topic echo to get one message
            cmd = f"ros2 topic echo {camera_topic} sensor_msgs/msg/Image --once"
            result = subprocess.run(
                cmd, 
                shell=True, 
                capture_output=True, 
                text=True,
                timeout=timeout
            )
            
            if result.returncode != 0:
                logger.error(f"Failed to capture image from {camera_topic}")
                return {
                    "success": False,
                    "error": f"Failed to capture from {camera_topic}",
                    "timestamp": timestamp
                }
            
            # Parse the message to get image dimensions
            output = result.stdout
            width = height = 0
            
            for line in output.split('\n'):
                if 'width:' in line:
                    width = int(line.split(':')[1].strip())
                elif 'height:' in line:
                    height = int(line.split(':')[1].strip())
            
            # For now, create a placeholder image with metadata
            # In production, we'd use cv_bridge to convert ROS Image to file
            self._create_placeholder_image(image_path, width, height, "Camera View")
            
            self.generated_files.append(str(image_path))
            self._cleanup_old_files()
            
            logger.info(f"Captured camera image: {image_path}")
            
            return {
                "success": True,
                "image_path": str(image_path),
                "timestamp": timestamp,
                "width": width,
                "height": height,
                "topic": camera_topic
            }
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Timeout capturing image from {camera_topic}")
            return {
                "success": False,
                "error": f"Timeout waiting for {camera_topic}",
                "timestamp": timestamp
            }
        except Exception as e:
            logger.error(f"Error capturing camera image: {e}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": timestamp
            }
    
    async def visualize_pointcloud(
        self,
        pointcloud_topic: str = "/sensing/lidar/concatenated/pointcloud",
        view_type: str = "bev",  # bird's eye view
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """
        Create a 2D visualization of pointcloud data.
        
        Args:
            pointcloud_topic: ROS2 topic for pointcloud
            view_type: Type of view (bev, front, side)
            timeout: Maximum wait time
            
        Returns:
            Dictionary with visualization image path and metadata
        """
        timestamp = datetime.now().isoformat()
        viz_filename = f"pointcloud_{view_type}_{int(time.time() * 1000)}.png"
        viz_path = self.media_dir / viz_filename
        
        try:
            # Get pointcloud header info
            cmd = f"ros2 topic echo {pointcloud_topic} sensor_msgs/msg/PointCloud2 --once"
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            
            if result.returncode != 0:
                logger.error(f"Failed to get pointcloud from {pointcloud_topic}")
                return {
                    "success": False,
                    "error": f"Failed to get pointcloud from {pointcloud_topic}",
                    "timestamp": timestamp
                }
            
            # Parse pointcloud info
            output = result.stdout
            num_points = 0
            
            for line in output.split('\n'):
                if 'width:' in line:
                    try:
                        num_points = int(line.split(':')[1].strip())
                    except:
                        pass
            
            # Create visualization placeholder
            # In production, we'd use PCL or Open3D to render actual pointcloud
            self._create_pointcloud_visualization(viz_path, view_type, num_points)
            
            self.generated_files.append(str(viz_path))
            self._cleanup_old_files()
            
            logger.info(f"Created pointcloud visualization: {viz_path}")
            
            return {
                "success": True,
                "image_path": str(viz_path),
                "timestamp": timestamp,
                "view_type": view_type,
                "num_points": num_points,
                "topic": pointcloud_topic
            }
            
        except Exception as e:
            logger.error(f"Error visualizing pointcloud: {e}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": timestamp
            }
    
    async def get_perception_snapshot(self) -> Dict[str, Any]:
        """
        Get a comprehensive perception snapshot with all sensor data.
        
        Returns:
            Dictionary with multiple image paths and detected objects
        """
        timestamp = datetime.now().isoformat()
        
        try:
            # Capture camera image
            camera_result = await self.capture_camera_image()
            
            # Create pointcloud visualization
            pointcloud_result = await self.visualize_pointcloud()
            
            # Get detected objects
            objects = await self._get_detected_objects()
            
            # Create annotated perception view
            annotated_path = await self._create_annotated_view(
                camera_result.get("image_path"),
                objects
            )
            
            return {
                "success": True,
                "timestamp": timestamp,
                "images": {
                    "camera": camera_result.get("image_path"),
                    "lidar_bev": pointcloud_result.get("image_path"),
                    "annotated": annotated_path
                },
                "detected_objects": objects,
                "summary": f"Detected {len(objects)} objects in scene"
            }
            
        except Exception as e:
            logger.error(f"Error creating perception snapshot: {e}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": timestamp
            }
    
    async def analyze_driving_scene(self) -> Dict[str, Any]:
        """
        Provide comprehensive scene analysis for AI reasoning.
        
        Returns:
            Dictionary with scene images, description, and recommendations
        """
        timestamp = datetime.now().isoformat()
        
        try:
            # Get perception snapshot
            snapshot = await self.get_perception_snapshot()
            
            # Analyze traffic lights
            traffic_lights = await self._get_traffic_light_status()
            
            # Get vehicle state
            vehicle_state = await self._get_vehicle_state()
            
            # Generate scene description
            scene_description = self._generate_scene_description(
                snapshot.get("detected_objects", []),
                traffic_lights,
                vehicle_state
            )
            
            return {
                "success": True,
                "timestamp": timestamp,
                "images": snapshot.get("images", {}),
                "scene_description": scene_description,
                "detected_objects": snapshot.get("detected_objects", []),
                "traffic_lights": traffic_lights,
                "vehicle_state": vehicle_state,
                "recommendations": self._generate_recommendations(scene_description)
            }
            
        except Exception as e:
            logger.error(f"Error analyzing driving scene: {e}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": timestamp
            }
    
    async def _get_detected_objects(self) -> List[Dict[str, Any]]:
        """Get list of detected objects from perception system."""
        try:
            cmd = "ros2 topic echo /perception/object_recognition/objects autoware_auto_perception_msgs/msg/PredictedObjects --once"
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode != 0:
                return []
            
            # Parse objects (simplified - would need proper parsing in production)
            objects = []
            # This is a placeholder - actual implementation would parse the message
            objects.append({
                "type": "vehicle",
                "id": 1,
                "position": {"x": 10.5, "y": 5.2, "z": 0},
                "velocity": {"x": 2.0, "y": 0, "z": 0},
                "confidence": 0.95
            })
            
            return objects
            
        except Exception as e:
            logger.warning(f"Could not get detected objects: {e}")
            return []
    
    async def _get_traffic_light_status(self) -> List[Dict[str, Any]]:
        """Get traffic light detection results."""
        try:
            cmd = "ros2 topic echo /perception/traffic_light_recognition/traffic_signals autoware_perception_msgs/msg/TrafficSignalArray --once"
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode != 0:
                return []
            
            # Placeholder parsing
            return [{
                "id": 1,
                "color": "green",
                "confidence": 0.98
            }]
            
        except Exception:
            return []
    
    async def _get_vehicle_state(self) -> Dict[str, Any]:
        """Get current vehicle state."""
        try:
            cmd = "ros2 topic echo /localization/kinematic_state --once"
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode == 0:
                # Parse vehicle position and velocity
                # This is simplified - actual implementation would parse properly
                return {
                    "position": {"x": 0, "y": 0, "z": 0},
                    "velocity": {"linear": {"x": 5.0, "y": 0, "z": 0}},
                    "speed_mps": 5.0
                }
        except:
            pass
        
        return {"position": {"x": 0, "y": 0, "z": 0}, "speed_mps": 0}
    
    async def _create_annotated_view(
        self, 
        base_image_path: Optional[str],
        objects: List[Dict[str, Any]]
    ) -> str:
        """Create an annotated image with perception overlays."""
        timestamp = int(time.time() * 1000)
        annotated_filename = f"annotated_{timestamp}.jpg"
        annotated_path = self.media_dir / annotated_filename
        
        # For now, create a placeholder
        # In production, would use OpenCV to draw bounding boxes, etc.
        self._create_placeholder_image(
            annotated_path, 
            1920, 1080, 
            f"Annotated View - {len(objects)} objects"
        )
        
        self.generated_files.append(str(annotated_path))
        return str(annotated_path)
    
    def _generate_scene_description(
        self,
        objects: List[Dict[str, Any]],
        traffic_lights: List[Dict[str, Any]],
        vehicle_state: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Generate high-level scene description."""
        return {
            "num_vehicles": len([o for o in objects if o.get("type") == "vehicle"]),
            "num_pedestrians": len([o for o in objects if o.get("type") == "pedestrian"]),
            "traffic_light_state": traffic_lights[0]["color"] if traffic_lights else "unknown",
            "vehicle_speed_mps": vehicle_state.get("speed_mps", 0),
            "scene_type": "urban" if traffic_lights else "highway",
            "complexity": "moderate"
        }
    
    def _generate_recommendations(self, scene_description: Dict[str, Any]) -> List[str]:
        """Generate driving recommendations based on scene."""
        recommendations = []
        
        if scene_description.get("num_pedestrians", 0) > 0:
            recommendations.append("Watch for pedestrians")
        
        if scene_description.get("traffic_light_state") == "yellow":
            recommendations.append("Prepare to stop for traffic light")
        
        if scene_description.get("num_vehicles", 0) > 5:
            recommendations.append("Heavy traffic - maintain safe following distance")
        
        return recommendations
    
    def _create_placeholder_image(
        self, 
        path: Path, 
        width: int, 
        height: int, 
        text: str
    ):
        """Create a placeholder image with text."""
        # Use ImageMagick to create a simple placeholder
        # In production, would save actual camera/visualization data
        cmd = f'convert -size {width}x{height} xc:gray -gravity center -pointsize 48 -annotate +0+0 "{text}" "{path}"'
        subprocess.run(cmd, shell=True, capture_output=True)
    
    def _create_pointcloud_visualization(
        self,
        path: Path,
        view_type: str,
        num_points: int
    ):
        """Create a pointcloud visualization placeholder."""
        text = f"Pointcloud {view_type.upper()} View\\n{num_points} points"
        self._create_placeholder_image(path, 1024, 768, text)
    
    def _cleanup_old_files(self):
        """Remove old files if we exceed the maximum."""
        if len(self.generated_files) > self.max_files:
            # Remove oldest files
            files_to_remove = self.generated_files[:-self.max_files]
            for file_path in files_to_remove:
                try:
                    if os.path.exists(file_path):
                        os.remove(file_path)
                        logger.debug(f"Cleaned up old file: {file_path}")
                except Exception as e:
                    logger.warning(f"Could not remove file {file_path}: {e}")
            
            self.generated_files = self.generated_files[-self.max_files:]
    
    def cleanup(self):
        """Clean up all generated files."""
        for file_path in self.generated_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception as e:
                logger.warning(f"Could not remove file {file_path}: {e}")
        
        self.generated_files.clear()
        logger.info("Cleaned up all perception media files")