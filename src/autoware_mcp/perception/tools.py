"""
MCP tools for perception data management.
"""

import asyncio
import json
import base64
from pathlib import Path
from typing import Dict, Any, Optional, List
from datetime import datetime
import numpy as np
import cv2

from .context import PerceptionContext
from .projections import ProjectionEngine
from .coloring import ColoringEngine


# Active perception contexts
_active_contexts: Dict[str, PerceptionContext] = {}


async def create_perception_context(
    context_type: str = "general",
    buffer_duration: float = 30.0,
    storage_path: Optional[str] = None
) -> Dict[str, Any]:
    """
    Create a new perception context for data capture.
    
    Args:
        context_type: Type of context (general, highway, parking, emergency)
        buffer_duration: Rolling buffer duration in seconds
        storage_path: Optional custom storage path
        
    Returns:
        Context information including ID
    """
    # Create new context
    storage = Path(storage_path) if storage_path else None
    context = PerceptionContext(
        buffer_duration=buffer_duration,
        storage_path=storage,
        context_type=context_type
    )
    
    # Start the context
    await context.start()
    
    # Store in active contexts
    _active_contexts[context.context_id] = context
    
    return {
        "success": True,
        "context_id": context.context_id,
        "context_type": context_type,
        "buffer_duration": buffer_duration,
        "created_at": datetime.now().isoformat(),
        "metadata": context.get_metadata()
    }


async def capture_sensors(
    context_id: str,
    topics: List[str],
    duration: float = 5.0
) -> Dict[str, Any]:
    """
    Capture data from specified sensor topics.
    
    Args:
        context_id: Context to capture into
        topics: List of ROS2 topics to capture from
        duration: Duration to capture for
        
    Returns:
        Capture statistics
    """
    if context_id not in _active_contexts:
        return {
            "success": False,
            "error": f"Context {context_id} not found"
        }
        
    context = _active_contexts[context_id]
    
    # Lazy import to avoid cv_bridge issues during testing
    try:
        from .ros2_bridge import get_bridge_node
        bridge_node = get_bridge_node()
    except (ImportError, AttributeError):
        bridge_node = None
    
    if not bridge_node:
        return {
            "success": False,
            "error": "ROS2 bridge not available"
        }
    
    # Track capture statistics
    capture_stats = {
        topic: {"count": 0, "first_timestamp": None, "last_timestamp": None}
        for topic in topics
    }
    
    # Define capture callback
    def capture_callback(topic: str):
        def callback(data: Dict):
            # Add to context buffer
            context.add_capture(data)
            
            # Update statistics
            stats = capture_stats[topic]
            stats["count"] += 1
            if stats["first_timestamp"] is None:
                stats["first_timestamp"] = data.get("timestamp_ns", 0)
            stats["last_timestamp"] = data.get("timestamp_ns", 0)
        return callback
    
    # Subscribe to topics
    for topic in topics:
        # Determine topic type
        topic_type = bridge_node.get_topic_type(topic)
        
        if not topic_type:
            capture_stats[topic]["error"] = "Topic not found"
            continue
            
        # Add subscription based on type
        if "Image" in topic_type:
            bridge_node.add_image_subscription(
                topic,
                capture_callback(topic)
            )
        elif "PointCloud2" in topic_type:
            bridge_node.add_pointcloud_subscription(
                topic,
                capture_callback(topic)
            )
        else:
            capture_stats[topic]["error"] = f"Unsupported type: {topic_type}"
    
    # Capture for specified duration
    start_time = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_time < duration:
        await asyncio.sleep(0.1)
        
    # Unsubscribe from topics
    for topic in topics:
        bridge_node.remove_subscription(topic)
    
    return {
        "success": True,
        "context_id": context_id,
        "duration": duration,
        "statistics": capture_stats,
        "buffer_stats": context.get_buffer_stats()
    }


async def generate_visualization(
    context_id: str,
    visualization_type: str,
    topic: Optional[str] = None,
    timestamp: Optional[float] = None,
    projection: str = "bev",
    color_mode: str = "height",
    scale: float = 0.1,
    projection_params: Optional[Dict] = None,
    color_params: Optional[Dict] = None
) -> Dict[str, Any]:
    """
    Generate visualization from captured data.
    
    Args:
        context_id: Context to visualize from
        visualization_type: Type of visualization (pointcloud, image, overlay)
        topic: Optional topic to filter by
        timestamp: Optional timestamp to visualize at
        projection: Projection type for point clouds
        color_mode: Coloring mode for point clouds
        scale: Scale for projections (meters per pixel)
        projection_params: Optional projection parameters
        color_params: Optional coloring parameters
        
    Returns:
        Visualization result with base64 encoded image
    """
    if context_id not in _active_contexts:
        return {
            "success": False,
            "error": f"Context {context_id} not found"
        }
        
    context = _active_contexts[context_id]
    
    # Get data to visualize
    if timestamp:
        capture = context.get_at_time(timestamp)
    else:
        capture = context.get_latest(topic)
        
    if not capture:
        return {
            "success": False,
            "error": "No data available for visualization"
        }
    
    # Generate visualization based on type
    if visualization_type == "pointcloud" and capture.get("type") == "pointcloud":
        # Use projection engine
        projection_engine = ProjectionEngine()
        points = capture.get("data")
        
        if points is None or len(points) == 0:
            return {
                "success": False,
                "error": "No point cloud data in capture"
            }
        
        # Generate projection
        from .projections import ProjectionParams
        params = ProjectionParams(**(projection_params or {}))
        
        img = projection_engine.project_pointcloud(
            points,
            projection,
            params,
            scale,
            color_mode,
            color_params
        )
        
        # Add legend if applicable
        if color_mode in ["height", "distance", "intensity"]:
            coloring_engine = ColoringEngine()
            legend = coloring_engine.create_legend(color_mode, color_params or {})
            # Overlay legend on image
            img[10:10+legend.shape[0], 10:10+legend.shape[1]] = legend
        
    elif visualization_type == "image" and capture.get("type") == "image":
        # Return captured image
        img = capture.get("data")
        
        if img is None:
            return {
                "success": False,
                "error": "No image data in capture"
            }
            
    elif visualization_type == "overlay":
        # Overlay visualization (future implementation)
        return {
            "success": False,
            "error": "Overlay visualization not yet implemented"
        }
    else:
        return {
            "success": False,
            "error": f"Invalid visualization type or data mismatch"
        }
    
    # Encode image to base64
    _, buffer = cv2.imencode('.png', img)
    img_base64 = base64.b64encode(buffer).decode('utf-8')
    
    # Save visualization to storage
    storage_path = context.storage_manager.save_visualization(
        img,
        context_id,
        f"{visualization_type}_{projection}" if visualization_type == "pointcloud" else visualization_type,
        {
            "topic": capture.get("topic"),
            "timestamp_ns": capture.get("timestamp_ns"),
            "projection": projection,
            "color_mode": color_mode,
            "scale": scale
        }
    )
    
    return {
        "success": True,
        "context_id": context_id,
        "visualization_type": visualization_type,
        "image_base64": img_base64,
        "storage_path": storage_path,
        "capture_info": {
            "topic": capture.get("topic"),
            "timestamp": capture.get("timestamp_ns", 0) / 1e9,
            "type": capture.get("type")
        }
    }


async def analyze_temporal(
    context_id: str,
    analysis_type: str,
    duration: float = 10.0,
    parameters: Optional[Dict] = None
) -> Dict[str, Any]:
    """
    Analyze temporal patterns in captured data.
    
    Args:
        context_id: Context to analyze
        analysis_type: Type of analysis (motion, frequency, synchronization)
        duration: Duration to analyze
        parameters: Analysis-specific parameters
        
    Returns:
        Analysis results
    """
    if context_id not in _active_contexts:
        return {
            "success": False,
            "error": f"Context {context_id} not found"
        }
        
    context = _active_contexts[context_id]
    params = parameters or {}
    
    # Get historical data
    captures = context.get_history(duration)
    
    if not captures:
        return {
            "success": False,
            "error": "No data available for analysis"
        }
    
    results = {}
    
    if analysis_type == "motion":
        # Analyze motion patterns
        pointcloud_captures = [c for c in captures if c.get("type") == "pointcloud"]
        
        if len(pointcloud_captures) < 2:
            return {
                "success": False,
                "error": "Insufficient point cloud data for motion analysis"
            }
        
        # Simple motion detection: compare consecutive frames
        motion_scores = []
        for i in range(1, len(pointcloud_captures)):
            prev_points = pointcloud_captures[i-1].get("data", np.array([]))
            curr_points = pointcloud_captures[i].get("data", np.array([]))
            
            if len(prev_points) > 0 and len(curr_points) > 0:
                # Calculate centroid change
                prev_centroid = np.mean(prev_points[:, :3], axis=0)
                curr_centroid = np.mean(curr_points[:, :3], axis=0)
                motion = np.linalg.norm(curr_centroid - prev_centroid)
                motion_scores.append(motion)
        
        results = {
            "motion_detected": max(motion_scores) > params.get("threshold", 0.5) if motion_scores else False,
            "max_motion": float(max(motion_scores)) if motion_scores else 0.0,
            "avg_motion": float(np.mean(motion_scores)) if motion_scores else 0.0,
            "num_frames": len(pointcloud_captures)
        }
        
    elif analysis_type == "frequency":
        # Analyze capture frequency
        topic_frequencies = {}
        
        for topic in set(c.get("topic") for c in captures):
            topic_captures = [c for c in captures if c.get("topic") == topic]
            if len(topic_captures) > 1:
                timestamps = [c.get("timestamp_ns", 0) / 1e9 for c in topic_captures]
                time_diffs = np.diff(timestamps)
                avg_period = np.mean(time_diffs) if len(time_diffs) > 0 else 0
                frequency = 1.0 / avg_period if avg_period > 0 else 0
                
                topic_frequencies[topic] = {
                    "frequency_hz": float(frequency),
                    "count": len(topic_captures),
                    "avg_period_ms": float(avg_period * 1000)
                }
        
        results = topic_frequencies
        
    elif analysis_type == "synchronization":
        # Analyze sensor synchronization
        topics = list(set(c.get("topic") for c in captures))
        sync_tolerance = params.get("tolerance_ms", 10.0) / 1000.0  # Convert to seconds
        
        # Group captures by approximate time
        time_groups = []
        for capture in captures:
            timestamp = capture.get("timestamp_ns", 0) / 1e9
            
            # Find matching time group
            matched = False
            for group in time_groups:
                if abs(group["timestamp"] - timestamp) < sync_tolerance:
                    group["captures"].append(capture)
                    matched = True
                    break
                    
            if not matched:
                time_groups.append({
                    "timestamp": timestamp,
                    "captures": [capture]
                })
        
        # Count synchronized captures
        fully_synced = sum(1 for g in time_groups if len(g["captures"]) == len(topics))
        partially_synced = sum(1 for g in time_groups if len(g["captures"]) > 1)
        
        results = {
            "topics": topics,
            "sync_tolerance_ms": params.get("tolerance_ms", 10.0),
            "total_groups": len(time_groups),
            "fully_synchronized": fully_synced,
            "partially_synchronized": partially_synced,
            "sync_percentage": (fully_synced / len(time_groups) * 100) if time_groups else 0
        }
    else:
        return {
            "success": False,
            "error": f"Unknown analysis type: {analysis_type}"
        }
    
    return {
        "success": True,
        "context_id": context_id,
        "analysis_type": analysis_type,
        "duration": duration,
        "results": results
    }


async def get_multi_sensor_view(
    context_id: str,
    timestamp: Optional[float] = None,
    layout: str = "grid"
) -> Dict[str, Any]:
    """
    Get synchronized multi-sensor view.
    
    Args:
        context_id: Context to get data from
        timestamp: Optional timestamp for synchronization
        layout: Layout type (grid, horizontal, vertical)
        
    Returns:
        Multi-sensor view information
    """
    if context_id not in _active_contexts:
        return {
            "success": False,
            "error": f"Context {context_id} not found"
        }
        
    context = _active_contexts[context_id]
    
    # Get captures around timestamp
    if timestamp:
        # Get captures within 100ms window
        window = 0.1
        captures = context.rolling_buffer.get_range(
            timestamp - window,
            timestamp + window
        )
    else:
        # Get latest captures for each topic
        all_captures = context.get_history(1.0)  # Last second
        
        # Group by topic and get latest for each
        topic_latest = {}
        for capture in all_captures:
            topic = capture.get("topic")
            if topic:
                if topic not in topic_latest or \
                   capture.get("timestamp_ns", 0) > topic_latest[topic].get("timestamp_ns", 0):
                    topic_latest[topic] = capture
        
        captures = list(topic_latest.values())
    
    if not captures:
        return {
            "success": False,
            "error": "No captures available"
        }
    
    # Organize captures by type
    sensor_data = {
        "images": [],
        "pointclouds": [],
        "timestamp": timestamp or (captures[0].get("timestamp_ns", 0) / 1e9 if captures else 0)
    }
    
    for capture in captures:
        if capture.get("type") == "image":
            sensor_data["images"].append({
                "topic": capture.get("topic"),
                "timestamp": capture.get("timestamp_ns", 0) / 1e9,
                "dimensions": capture.get("dimensions"),
                "frame_id": capture.get("frame_id")
            })
        elif capture.get("type") == "pointcloud":
            sensor_data["pointclouds"].append({
                "topic": capture.get("topic"),
                "timestamp": capture.get("timestamp_ns", 0) / 1e9,
                "num_points": capture.get("num_points"),
                "frame_id": capture.get("frame_id")
            })
    
    return {
        "success": True,
        "context_id": context_id,
        "layout": layout,
        "sensor_data": sensor_data,
        "num_sensors": len(sensor_data["images"]) + len(sensor_data["pointclouds"])
    }


async def save_context_buffer(
    context_id: str,
    duration: Optional[float] = None,
    trigger: Optional[str] = None
) -> Dict[str, Any]:
    """
    Save context buffer to persistent storage.
    
    Args:
        context_id: Context to save
        duration: Optional duration to save (default: all)
        trigger: Optional trigger event name
        
    Returns:
        Archive information
    """
    if context_id not in _active_contexts:
        return {
            "success": False,
            "error": f"Context {context_id} not found"
        }
        
    context = _active_contexts[context_id]
    
    try:
        # Save buffer
        archive_path = await context.save_buffer(duration, trigger)
        
        # Get archive size
        archive_size = Path(archive_path).stat().st_size if Path(archive_path).exists() else 0
        
        return {
            "success": True,
            "context_id": context_id,
            "archive_path": archive_path,
            "archive_size_bytes": archive_size,
            "duration_saved": duration,
            "trigger": trigger
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }


async def cleanup_contexts(
    max_age_hours: Optional[float] = None,
    max_size_gb: Optional[float] = None
) -> Dict[str, Any]:
    """
    Clean up old contexts and storage.
    
    Args:
        max_age_hours: Remove contexts older than this
        max_size_gb: Remove oldest contexts if total size exceeds this
        
    Returns:
        Cleanup statistics
    """
    # Get any active context for storage manager access
    if _active_contexts:
        context = next(iter(_active_contexts.values()))
        storage_manager = context.storage_manager
    else:
        # Create temporary context for storage access
        from .storage import StorageManager
        storage_manager = StorageManager()
    
    removed_by_age = 0
    removed_by_size = 0
    
    if max_age_hours:
        removed_by_age = storage_manager.cleanup_old_contexts(max_age_hours)
        
    if max_size_gb:
        removed_by_size = storage_manager.cleanup_by_size(max_size_gb)
    
    return {
        "success": True,
        "removed_by_age": removed_by_age,
        "removed_by_size": removed_by_size,
        "total_removed": removed_by_age + removed_by_size,
        "active_contexts": list(_active_contexts.keys())
    }


async def list_contexts() -> Dict[str, Any]:
    """
    List all active and stored contexts.
    
    Returns:
        Context listing
    """
    # Get storage manager
    if _active_contexts:
        context = next(iter(_active_contexts.values()))
        storage_manager = context.storage_manager
    else:
        from .storage import StorageManager
        storage_manager = StorageManager()
    
    # Get active contexts
    active = []
    for ctx_id, ctx in _active_contexts.items():
        active.append({
            "context_id": ctx_id,
            "context_type": ctx.context_type,
            "buffer_stats": ctx.get_buffer_stats(),
            "metadata": ctx.get_metadata()
        })
    
    # Get stored contexts
    stored = storage_manager.list_contexts()
    
    return {
        "success": True,
        "active_contexts": active,
        "stored_contexts": stored,
        "total_active": len(active),
        "total_stored": len(stored)
    }