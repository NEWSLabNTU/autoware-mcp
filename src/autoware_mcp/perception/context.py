"""
Perception context management.
"""

import time
import asyncio
from pathlib import Path
from typing import Optional, Dict, Any, List
from datetime import datetime
import uuid

from .buffer import CircularBuffer
from .storage import StorageManager


class PerceptionContext:
    """Manages the lifecycle of perception data capture and analysis."""
    
    def __init__(
        self,
        buffer_duration: float = 30.0,
        storage_path: Optional[Path] = None,
        context_type: str = "general"
    ):
        """
        Initialize perception context.
        
        Args:
            buffer_duration: Rolling buffer size in seconds
            storage_path: Optional custom storage path
            context_type: Type of context (general, highway, parking, emergency)
        """
        # Generate unique context ID
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        unique_id = str(uuid.uuid4())[:8]
        self.context_id = f"ctx_{context_type}_{timestamp}_{unique_id}"
        
        # Initialize components
        self.context_type = context_type
        self.rolling_buffer = CircularBuffer(duration=buffer_duration)
        self.storage_manager = StorageManager(base_path=storage_path)
        self.active_subscriptions = {}
        self.metadata = {
            "context_id": self.context_id,
            "context_type": context_type,
            "buffer_duration": buffer_duration,
            "created_at": time.time(),
            "created_at_str": datetime.now().isoformat()
        }
        
        # State management
        self.is_active = False
        self.should_save = False
        self._capture_task = None
        
    async def __aenter__(self):
        """Start context with automatic buffer management."""
        await self.start()
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Clean up and optionally save buffer."""
        await self.stop()
        if self.should_save:
            await self.save_buffer()
            
    async def start(self):
        """Start the perception context."""
        self.is_active = True
        self.metadata["started_at"] = time.time()
        
    async def stop(self):
        """Stop the perception context."""
        self.is_active = False
        self.metadata["stopped_at"] = time.time()
        
        # Cancel any active capture tasks
        if self._capture_task and not self._capture_task.done():
            self._capture_task.cancel()
            
    def add_capture(self, data: Dict[str, Any]) -> None:
        """
        Add a capture to the buffer.
        
        Args:
            data: Capture data with type, topic, and sensor data
        """
        if not self.is_active:
            raise RuntimeError("Context is not active")
            
        # Ensure timestamp
        if 'timestamp_ns' not in data:
            data['timestamp_ns'] = time.time_ns()
            
        # Add context ID
        data['context_id'] = self.context_id
        
        # Add to buffer
        self.rolling_buffer.add(data)
        
    def get_latest(self, topic: Optional[str] = None) -> Optional[Dict]:
        """
        Get latest capture, optionally filtered by topic.
        
        Args:
            topic: Optional topic to filter by
            
        Returns:
            Latest capture or None
        """
        captures = self.rolling_buffer.get_latest(100)  # Get last 100
        
        if topic:
            # Filter by topic
            for capture in reversed(captures):
                if capture.get('topic') == topic:
                    return capture
            return None
        else:
            # Return most recent
            return captures[-1] if captures else None
            
    def get_history(self, duration: float) -> List[Dict]:
        """
        Get historical data from buffer.
        
        Args:
            duration: Duration in seconds to retrieve
            
        Returns:
            List of captures within duration
        """
        end_time = time.time()
        start_time = end_time - duration
        return self.rolling_buffer.get_range(start_time, end_time)
        
    def get_at_time(self, timestamp: float) -> Optional[Dict]:
        """
        Get capture closest to specified time.
        
        Args:
            timestamp: Unix timestamp in seconds
            
        Returns:
            Closest capture or None
        """
        # Get captures around the timestamp
        window = 1.0  # 1 second window
        captures = self.rolling_buffer.get_range(
            timestamp - window,
            timestamp + window
        )
        
        if not captures:
            return None
            
        # Find closest
        closest = min(
            captures,
            key=lambda c: abs(c['timestamp_ns'] / 1e9 - timestamp)
        )
        return closest
        
    async def save_buffer(self, duration: Optional[float] = None,
                         trigger: Optional[str] = None) -> str:
        """
        Save buffer contents to storage.
        
        Args:
            duration: Optional duration to save (default: all)
            trigger: Optional trigger event name
            
        Returns:
            Archive path
        """
        # Get data to save
        if duration:
            data = self.get_history(duration)
        else:
            data = self.rolling_buffer.get_all()
            
        # Save each capture
        for capture in data:
            if capture.get('type') == 'image' and 'data' in capture:
                self.storage_manager.save_image(
                    capture['data'],
                    self.context_id,
                    capture
                )
            elif capture.get('type') == 'pointcloud' and 'data' in capture:
                self.storage_manager.save_pointcloud(
                    capture['data'],
                    self.context_id,
                    capture
                )
                
        # Archive the context
        archive_path = self.storage_manager.archive_context(
            self.context_id,
            trigger
        )
        
        return archive_path
        
    def get_metadata(self) -> Dict[str, Any]:
        """Get context metadata."""
        return self.metadata.copy()
        
    def set_save_on_exit(self, should_save: bool = True):
        """Set whether to save buffer on context exit."""
        self.should_save = should_save
        
    def get_buffer_stats(self) -> Dict[str, Any]:
        """Get buffer statistics."""
        return {
            "num_items": self.rolling_buffer.size(),
            "size_bytes": self.rolling_buffer.size_bytes(),
            "duration": self.rolling_buffer.duration,
            "context_id": self.context_id,
            "is_active": self.is_active
        }