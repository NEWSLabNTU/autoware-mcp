#!/usr/bin/env python3
"""
Demonstration of Perception Media Bridge tools.

This example shows how AI agents can access Autoware's perception data
through the MCP server's perception tools.
"""

import asyncio
import json
from typing import Dict, Any
import subprocess


class PerceptionDemo:
    """Demo client for perception tools using MCP server."""

    async def capture_camera_views(self):
        """Capture images from all available cameras."""
        print("\n=== Capturing Camera Views ===")
        
        cameras = ["front", "rear", "left", "right", "traffic_light"]
        
        for camera in cameras:
            print(f"\nCapturing {camera} camera...")
            # In a real scenario, this would call the MCP tool
            # For demo, we simulate the response
            result = {
                "success": True,
                "image_path": f"/tmp/autoware_mcp/perception/camera_{camera}_demo.jpg",
                "camera": camera,
                "dimensions": "1920x1080",
                "message": f"Camera image saved. Use Read tool to view."
            }
            
            if result["success"]:
                print(f"  ✓ Captured: {result['image_path']}")
                print(f"    Resolution: {result['dimensions']}")
            else:
                print(f"  ✗ Failed to capture {camera} camera")

    async def visualize_lidar_data(self):
        """Create LiDAR visualizations from different viewpoints."""
        print("\n=== Visualizing LiDAR Data ===")
        
        view_types = ["bev", "front", "side", "rear"]
        
        for view in view_types:
            print(f"\nCreating {view} visualization...")
            # Simulated MCP response
            result = {
                "success": True,
                "image_path": f"/tmp/autoware_mcp/perception/lidar_{view}_demo.png",
                "view_type": view,
                "num_points": 50000,
                "message": f"LiDAR visualization saved. Use Read tool to view."
            }
            
            if result["success"]:
                print(f"  ✓ Created: {result['image_path']}")
                print(f"    Points: {result['num_points']:,}")
            else:
                print(f"  ✗ Failed to create {view} visualization")

    async def get_perception_snapshot(self):
        """Get comprehensive perception snapshot."""
        print("\n=== Getting Perception Snapshot ===")
        
        # Simulated MCP response
        result = {
            "success": True,
            "camera_image": "/tmp/autoware_mcp/perception/camera_snapshot.jpg",
            "lidar_visualization": "/tmp/autoware_mcp/perception/lidar_snapshot.png",
            "annotated_view": "/tmp/autoware_mcp/perception/annotated_snapshot.jpg",
            "detected_objects": [
                {"type": "vehicle", "id": 1, "position": {"x": 10.5, "y": 5.2}},
                {"type": "vehicle", "id": 2, "position": {"x": 25.3, "y": 8.7}},
                {"type": "pedestrian", "id": 3, "position": {"x": 15.0, "y": 2.1}},
            ],
            "num_objects": 3,
            "message": "Perception snapshot captured. Use Read tool to view images."
        }
        
        if result["success"]:
            print("  ✓ Snapshot captured successfully")
            print(f"\n  Images:")
            print(f"    - Camera: {result['camera_image']}")
            print(f"    - LiDAR: {result['lidar_visualization']}")
            print(f"    - Annotated: {result['annotated_view']}")
            print(f"\n  Detected {result['num_objects']} objects:")
            for obj in result['detected_objects']:
                print(f"    - {obj['type'].capitalize()} (ID: {obj['id']}) at ({obj['position']['x']:.1f}, {obj['position']['y']:.1f})")

    async def analyze_driving_scene(self):
        """Perform comprehensive scene analysis."""
        print("\n=== Analyzing Driving Scene ===")
        
        # Simulated MCP response
        result = {
            "success": True,
            "images": {
                "camera": "/tmp/autoware_mcp/perception/scene_camera.jpg",
                "lidar_bev": "/tmp/autoware_mcp/perception/scene_lidar.png",
                "annotated": "/tmp/autoware_mcp/perception/scene_annotated.jpg"
            },
            "scene_summary": {
                "num_vehicles": 5,
                "num_pedestrians": 2,
                "traffic_light": "green",
                "vehicle_speed_mps": 8.3,
                "scene_type": "urban"
            },
            "detected_objects": [
                {"type": "vehicle", "id": 1, "velocity": {"x": 5.0, "y": 0}},
                {"type": "pedestrian", "id": 2, "velocity": {"x": 0, "y": 1.2}}
            ],
            "traffic_lights": [
                {"id": 1, "color": "green", "confidence": 0.98}
            ],
            "recommendations": [
                "Maintain safe following distance",
                "Watch for pedestrian crossing",
                "Prepare for potential traffic light change"
            ],
            "message": "Scene analysis complete. Use Read tool to view images."
        }
        
        if result["success"]:
            print("  ✓ Scene analysis completed")
            
            print(f"\n  Scene Summary:")
            summary = result['scene_summary']
            print(f"    - Scene Type: {summary['scene_type']}")
            print(f"    - Vehicles: {summary['num_vehicles']}")
            print(f"    - Pedestrians: {summary['num_pedestrians']}")
            print(f"    - Traffic Light: {summary['traffic_light']}")
            print(f"    - Vehicle Speed: {summary['vehicle_speed_mps']:.1f} m/s")
            
            print(f"\n  Traffic Lights:")
            for light in result['traffic_lights']:
                print(f"    - Light {light['id']}: {light['color']} (confidence: {light['confidence']:.0%})")
            
            print(f"\n  AI Recommendations:")
            for i, rec in enumerate(result['recommendations'], 1):
                print(f"    {i}. {rec}")
            
            print(f"\n  Generated Images:")
            for img_type, path in result['images'].items():
                print(f"    - {img_type}: {path}")

    async def detect_objects_by_type(self):
        """Get detected objects filtered by type."""
        print("\n=== Detecting Objects by Type ===")
        
        object_types = ["all", "vehicle", "pedestrian"]
        
        for obj_type in object_types:
            print(f"\nDetecting {obj_type} objects...")
            
            # Simulated object detection
            if obj_type == "all":
                objects = [
                    {"type": "vehicle", "id": 1, "position": {"x": 10, "y": 5}},
                    {"type": "vehicle", "id": 2, "position": {"x": 20, "y": 8}},
                    {"type": "pedestrian", "id": 3, "position": {"x": 15, "y": 2}},
                    {"type": "bicycle", "id": 4, "position": {"x": 12, "y": 6}},
                ]
            elif obj_type == "vehicle":
                objects = [
                    {"type": "vehicle", "id": 1, "position": {"x": 10, "y": 5}},
                    {"type": "vehicle", "id": 2, "position": {"x": 20, "y": 8}},
                ]
            elif obj_type == "pedestrian":
                objects = [
                    {"type": "pedestrian", "id": 3, "position": {"x": 15, "y": 2}},
                ]
            else:
                objects = []
            
            result = {
                "success": True,
                "objects": objects,
                "count": len(objects),
                "object_type": obj_type,
                "message": f"Found {len(objects)} objects of type '{obj_type}'"
            }
            
            print(f"  ✓ Found {result['count']} {obj_type} objects")
            if result['objects']:
                for obj in result['objects']:
                    print(f"    - {obj['type']} (ID: {obj['id']}) at ({obj['position']['x']}, {obj['position']['y']})")

    async def run_demo(self):
        """Run complete perception demo."""
        print("=" * 60)
        print("       AUTOWARE PERCEPTION MEDIA BRIDGE DEMO")
        print("=" * 60)
        print("\nThis demo shows how AI agents can access Autoware's")
        print("perception data through MCP tools.")
        
        # Run all demos
        await self.capture_camera_views()
        await self.visualize_lidar_data()
        await self.get_perception_snapshot()
        await self.analyze_driving_scene()
        await self.detect_objects_by_type()
        
        print("\n" + "=" * 60)
        print("Demo complete!")
        print("\nIn a real scenario, these tools would:")
        print("  1. Connect to actual ROS2 topics")
        print("  2. Capture real sensor data")
        print("  3. Generate actual images for AI analysis")
        print("  4. Return file paths that AI agents can read")
        print("\nThe AI agent would then use the Read tool to")
        print("view the images and make driving decisions.")
        print("=" * 60)


async def main():
    """Main function to run the demo."""
    demo = PerceptionDemo()
    await demo.run_demo()


if __name__ == "__main__":
    asyncio.run(main())