"""
Storage management for perception data.
"""

import os
import json
import shutil
import time
from pathlib import Path
from typing import Dict, Any, Optional, List
import numpy as np
import cv2
from datetime import datetime


class StorageManager:
    """Manages perception data storage with automatic organization and cleanup."""
    
    def __init__(self, base_path: Optional[Path] = None):
        """
        Initialize storage manager.
        
        Args:
            base_path: Base directory for storage (default: /tmp/autoware_mcp/perception)
        """
        self.base_path = base_path or Path("/tmp/autoware_mcp/perception")
        self.structure = {
            "contexts": self.base_path / "contexts",  # Active contexts
            "archives": self.base_path / "archives",  # Saved contexts
            "cache": self.base_path / "cache"  # Pre-computed visualizations
        }
        
        # Create directory structure
        for path in self.structure.values():
            path.mkdir(parents=True, exist_ok=True)
            
    def get_context_path(self, context_id: str) -> Path:
        """
        Get storage path for a context.
        
        Args:
            context_id: Context identifier
            
        Returns:
            Path to context directory
        """
        path = self.structure["contexts"] / context_id
        path.mkdir(parents=True, exist_ok=True)
        return path
        
    def save_image(self, image: np.ndarray, context_id: str, metadata: Dict) -> str:
        """
        Save image data.
        
        Args:
            image: Image array (BGR format)
            context_id: Context identifier
            metadata: Image metadata including topic and timestamp
            
        Returns:
            Path to saved image file
        """
        # Generate filename
        timestamp_ns = metadata.get("timestamp_ns", time.time_ns())
        topic_safe = metadata.get("topic", "unknown").replace("/", "_")
        filename = f"image_{topic_safe}_{timestamp_ns}.jpg"
        
        # Create path
        context_path = self.get_context_path(context_id)
        image_dir = context_path / "images"
        image_dir.mkdir(exist_ok=True)
        filepath = image_dir / filename
        
        # Save image
        cv2.imwrite(str(filepath), image)
        
        # Save metadata
        meta_path = filepath.with_suffix('.json')
        with open(meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        return str(filepath)
        
    def save_pointcloud(self, points: np.ndarray, context_id: str, metadata: Dict) -> str:
        """
        Save point cloud data.
        
        Args:
            points: Point cloud array (Nx3 or Nx4)
            context_id: Context identifier
            metadata: Point cloud metadata
            
        Returns:
            Path to saved point cloud file
        """
        # Generate filename
        timestamp_ns = metadata.get("timestamp_ns", time.time_ns())
        topic_safe = metadata.get("topic", "unknown").replace("/", "_")
        filename = f"pointcloud_{topic_safe}_{timestamp_ns}.npy"
        
        # Create path
        context_path = self.get_context_path(context_id)
        pc_dir = context_path / "pointclouds"
        pc_dir.mkdir(exist_ok=True)
        filepath = pc_dir / filename
        
        # Save point cloud
        np.save(filepath, points)
        
        # Save metadata
        meta_path = filepath.with_suffix('.json')
        with open(meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        return str(filepath)
        
    def save_visualization(self, image: np.ndarray, context_id: str, 
                         viz_type: str, metadata: Dict) -> str:
        """
        Save visualization image.
        
        Args:
            image: Visualization image array
            context_id: Context identifier
            viz_type: Type of visualization (e.g., "bev", "front")
            metadata: Visualization metadata
            
        Returns:
            Path to saved visualization
        """
        # Generate filename
        timestamp_ns = metadata.get("timestamp_ns", time.time_ns())
        filename = f"viz_{viz_type}_{timestamp_ns}.png"
        
        # Create path
        context_path = self.get_context_path(context_id)
        viz_dir = context_path / "visualizations"
        viz_dir.mkdir(exist_ok=True)
        filepath = viz_dir / filename
        
        # Save visualization
        cv2.imwrite(str(filepath), image)
        
        # Save metadata
        meta_path = filepath.with_suffix('.json')
        with open(meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        return str(filepath)
        
    def archive_context(self, context_id: str, trigger: Optional[str] = None) -> str:
        """
        Archive a context for long-term storage.
        
        Args:
            context_id: Context to archive
            trigger: Optional trigger event name
            
        Returns:
            Path to archive file
        """
        context_path = self.get_context_path(context_id)
        if not context_path.exists():
            raise ValueError(f"Context {context_id} does not exist")
            
        # Generate archive name
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        trigger_str = f"_{trigger}" if trigger else ""
        archive_name = f"{context_id}{trigger_str}_{timestamp}.tar.gz"
        archive_path = self.structure["archives"] / archive_name
        
        # Create archive
        import tarfile
        with tarfile.open(archive_path, "w:gz") as tar:
            tar.add(context_path, arcname=context_id)
            
        # Remove original context
        shutil.rmtree(context_path)
        
        return str(archive_path)
        
    def cleanup_old_contexts(self, max_age_hours: float = 24.0) -> int:
        """
        Remove contexts older than specified age.
        
        Args:
            max_age_hours: Maximum age in hours
            
        Returns:
            Number of contexts removed
        """
        cutoff_time = time.time() - (max_age_hours * 3600)
        removed_count = 0
        
        for context_dir in self.structure["contexts"].iterdir():
            if not context_dir.is_dir():
                continue
                
            # Check modification time
            if context_dir.stat().st_mtime < cutoff_time:
                shutil.rmtree(context_dir)
                removed_count += 1
                
        return removed_count
        
    def cleanup_by_size(self, max_size_gb: float = 10.0) -> int:
        """
        Remove oldest contexts if total size exceeds limit.
        
        Args:
            max_size_gb: Maximum total size in GB
            
        Returns:
            Number of contexts removed
        """
        max_size_bytes = max_size_gb * 1024 * 1024 * 1024
        
        # Get all contexts with sizes
        contexts = []
        for context_dir in self.structure["contexts"].iterdir():
            if context_dir.is_dir():
                size = sum(f.stat().st_size for f in context_dir.rglob('*') if f.is_file())
                contexts.append((context_dir, size, context_dir.stat().st_mtime))
                
        # Sort by modification time (oldest first)
        contexts.sort(key=lambda x: x[2])
        
        # Calculate total size and remove if needed
        total_size = sum(c[1] for c in contexts)
        removed_count = 0
        
        for context_dir, size, _ in contexts:
            if total_size <= max_size_bytes:
                break
                
            shutil.rmtree(context_dir)
            total_size -= size
            removed_count += 1
            
        return removed_count
        
    def get_context_size(self, context_id: str) -> int:
        """
        Get size of a context in bytes.
        
        Args:
            context_id: Context identifier
            
        Returns:
            Size in bytes
        """
        context_path = self.get_context_path(context_id)
        if not context_path.exists():
            return 0
            
        return sum(f.stat().st_size for f in context_path.rglob('*') if f.is_file())
        
    def list_contexts(self) -> List[Dict[str, Any]]:
        """
        List all active contexts.
        
        Returns:
            List of context information dictionaries
        """
        contexts = []
        for context_dir in self.structure["contexts"].iterdir():
            if not context_dir.is_dir():
                continue
                
            info = {
                "context_id": context_dir.name,
                "path": str(context_dir),
                "size_bytes": self.get_context_size(context_dir.name),
                "modified": context_dir.stat().st_mtime
            }
            contexts.append(info)
            
        return contexts