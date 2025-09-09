"""
Circular buffer implementation for perception data.
"""

import collections
import time
from threading import RLock
from typing import List, Dict, Any, Optional, Tuple
import numpy as np


class CircularBuffer:
    """Thread-safe circular buffer for sensor data with automatic cleanup."""
    
    def __init__(self, duration: float = 30.0, max_size_bytes: int = 1024 * 1024 * 1024):
        """
        Initialize circular buffer.
        
        Args:
            duration: Maximum age of entries in seconds (default 30s)
            max_size_bytes: Maximum buffer size in bytes (default 1GB)
        """
        self.duration = duration
        self.max_size_bytes = max_size_bytes
        self.buffer = collections.deque()
        self.lock = RLock()
        self.current_size_bytes = 0
        
    def add(self, item: Dict[str, Any]) -> None:
        """
        Add item to buffer with automatic cleanup.
        
        Args:
            item: Data item to add (must be dict with optional 'timestamp_ns' field)
        """
        with self.lock:
            # Add timestamp if not present
            if 'timestamp_ns' not in item:
                item['timestamp_ns'] = time.time_ns()
            
            # Estimate item size
            item_size = self._estimate_size(item)
            
            # Clean old entries
            self._cleanup_old()
            
            # Make room if needed
            while self.current_size_bytes + item_size > self.max_size_bytes:
                if not self.buffer:
                    break
                removed = self.buffer.popleft()
                self.current_size_bytes -= self._estimate_size(removed)
            
            # Add new item
            self.buffer.append(item)
            self.current_size_bytes += item_size
            
    def get_range(self, start_time: float, end_time: float) -> List[Dict]:
        """
        Get items within time range.
        
        Args:
            start_time: Start time in seconds (Unix timestamp)
            end_time: End time in seconds (Unix timestamp)
            
        Returns:
            List of items within the time range
        """
        with self.lock:
            start_ns = int(start_time * 1e9)
            end_ns = int(end_time * 1e9)
            
            return [
                item for item in self.buffer
                if start_ns <= item['timestamp_ns'] <= end_ns
            ]
            
    def get_latest(self, n: int = 1) -> List[Dict]:
        """
        Get latest n items from buffer.
        
        Args:
            n: Number of items to retrieve
            
        Returns:
            List of latest items (newest last)
        """
        with self.lock:
            return list(self.buffer)[-n:] if self.buffer else []
            
    def get_all(self) -> List[Dict]:
        """Get all items in buffer."""
        with self.lock:
            return list(self.buffer)
            
    def clear(self) -> None:
        """Clear all items from buffer."""
        with self.lock:
            self.buffer.clear()
            self.current_size_bytes = 0
            
    def size(self) -> int:
        """Get number of items in buffer."""
        with self.lock:
            return len(self.buffer)
            
    def size_bytes(self) -> int:
        """Get estimated size in bytes."""
        with self.lock:
            return self.current_size_bytes
            
    def _cleanup_old(self) -> None:
        """Remove entries older than duration."""
        cutoff_ns = time.time_ns() - int(self.duration * 1e9)
        
        while self.buffer and self.buffer[0]['timestamp_ns'] < cutoff_ns:
            removed = self.buffer.popleft()
            self.current_size_bytes -= self._estimate_size(removed)
            
    def _estimate_size(self, item: Dict) -> int:
        """
        Estimate memory size of item in bytes.
        
        Args:
            item: Item to estimate size for
            
        Returns:
            Estimated size in bytes
        """
        if 'type' not in item:
            return 1024  # Default estimate
            
        if item['type'] == 'image':
            # BGR image size
            if 'dimensions' in item:
                h, w = item['dimensions']
                return h * w * 3
            return 1920 * 1080 * 3  # Default Full HD
            
        elif item['type'] == 'pointcloud':
            # XYZI float32 points
            if 'num_points' in item:
                return item['num_points'] * 16
            return 100000 * 16  # Default 100k points
            
        else:
            # Generic estimate
            return 1024