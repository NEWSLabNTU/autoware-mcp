"""
Point cloud projection engine for various visualization types.
"""

import numpy as np
import cv2
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass


@dataclass
class ProjectionParams:
    """Parameters for customizable projections."""
    
    # Angle parameters (for angled projection)
    azimuth: float = 0.0  # Horizontal angle in degrees (-180 to 180)
    elevation: float = 0.0  # Vertical angle in degrees (-90 to 90)
    
    # Perspective parameters
    fov_horizontal: float = 90.0  # Horizontal field of view
    fov_vertical: float = 60.0  # Vertical field of view
    camera_height: float = 1.5  # Camera height in meters
    look_ahead: float = 10.0  # Look-ahead distance in meters
    
    # View range parameters
    x_range: Tuple[float, float] = (-50, 50)  # X range in meters
    y_range: Tuple[float, float] = (-30, 30)  # Y range in meters
    z_range: Tuple[float, float] = (-2, 10)  # Z range for filtering
    
    # Common parameters
    clip_distance: float = 100.0  # Maximum distance to render
    ground_removal: bool = True  # Remove ground plane
    ground_threshold: float = -1.5  # Ground height threshold


# Convenient presets for common angles
PROJECTION_PRESETS = {
    "front_left_30": ProjectionParams(azimuth=30, elevation=0),
    "front_left_45": ProjectionParams(azimuth=45, elevation=0),
    "front_left_60": ProjectionParams(azimuth=60, elevation=0),
    "front_right_30": ProjectionParams(azimuth=-30, elevation=0),
    "front_right_45": ProjectionParams(azimuth=-45, elevation=0),
    "front_right_60": ProjectionParams(azimuth=-60, elevation=0),
    "rear_left_120": ProjectionParams(azimuth=120, elevation=0),
    "rear_left_135": ProjectionParams(azimuth=135, elevation=0),
    "rear_right_120": ProjectionParams(azimuth=-120, elevation=0),
    "rear_right_135": ProjectionParams(azimuth=-135, elevation=0),
    "elevated_front": ProjectionParams(azimuth=0, elevation=30),
    "elevated_bev": ProjectionParams(azimuth=0, elevation=80),
    "driver_view": ProjectionParams(azimuth=0, elevation=5, camera_height=1.2),
}


class ProjectionEngine:
    """Handles all projection types with customizable parameters."""
    
    def project_pointcloud(
        self,
        points: np.ndarray,
        projection: str,
        params: Optional[ProjectionParams] = None,
        scale: float = 0.1,
        color_mode: str = "height",
        color_params: Dict = None
    ) -> np.ndarray:
        """
        Project point cloud to 2D image with flexible parameters.
        
        Args:
            points: Point cloud array (Nx3 or Nx4)
            projection: Projection type (bev, front, angled, etc.)
            params: Projection parameters
            scale: Scale in meters per pixel
            color_mode: Coloring mode for points
            color_params: Parameters for coloring
            
        Returns:
            Projected image as numpy array
        """
        # Check for presets
        if projection in PROJECTION_PRESETS:
            params = PROJECTION_PRESETS[projection]
            projection = "angled"
        
        # Use default params if not provided
        if params is None:
            params = ProjectionParams()
        
        # Apply ground removal if requested
        if params.ground_removal and len(points) > 0:
            points = points[points[:, 2] > params.ground_threshold]
        
        # Apply distance clipping
        if len(points) > 0:
            distances = np.linalg.norm(points[:, :3], axis=1)
            points = points[distances < params.clip_distance]
        
        # Handle empty point cloud
        if len(points) == 0:
            # Return empty image
            width = int((params.x_range[1] - params.x_range[0]) / scale)
            height = int((params.y_range[1] - params.y_range[0]) / scale)
            return np.zeros((height, width, 3), dtype=np.uint8)
        
        # Import coloring engine
        from .coloring import ColoringEngine
        coloring_engine = ColoringEngine()
        
        # Route to appropriate projection method
        if projection == "bev":
            return self._project_bev(points, params, scale, coloring_engine, color_mode, color_params)
        elif projection == "angled":
            return self._project_angled(points, params, scale, coloring_engine, color_mode, color_params)
        elif projection == "perspective":
            return self._project_perspective(points, params, scale, coloring_engine, color_mode, color_params)
        elif projection in ["front", "rear", "left", "right"]:
            return self._project_orthographic(points, projection, params, scale, coloring_engine, color_mode, color_params)
        else:
            # Default to BEV
            return self._project_bev(points, params, scale, coloring_engine, color_mode, color_params)
            
    def _project_bev(
        self,
        points: np.ndarray,
        params: ProjectionParams,
        scale: float,
        coloring_engine,
        color_mode: str,
        color_params: Dict
    ) -> np.ndarray:
        """Bird's eye view projection with configurable coloring."""
        # Calculate image dimensions
        x_range = params.x_range
        y_range = params.y_range
        
        width = int((x_range[1] - x_range[0]) / scale)
        height = int((y_range[1] - y_range[0]) / scale)
        
        # Create image
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Project points
        x_img = ((points[:, 0] - x_range[0]) / scale).astype(int)
        y_img = ((points[:, 1] - y_range[0]) / scale).astype(int)
        
        # Filter valid indices
        valid = (x_img >= 0) & (x_img < width) & (y_img >= 0) & (y_img < height)
        x_img = x_img[valid]
        y_img = y_img[valid]
        points_valid = points[valid]
        
        if len(points_valid) == 0:
            return img
        
        # Apply coloring based on mode
        colors = coloring_engine.apply_coloring(points_valid, color_mode, color_params or {})
        
        # Draw points with Z-buffering for occlusion
        z_buffer = np.full((height, width), np.inf)
        for x, y, z, color in zip(x_img, y_img, points_valid[:, 2], colors):
            if z < z_buffer[height - y - 1, x]:
                z_buffer[height - y - 1, x] = z
                cv2.circle(img, (x, height - y - 1), 1, color.tolist(), -1)
        
        return img
        
    def _project_angled(
        self,
        points: np.ndarray,
        params: ProjectionParams,
        scale: float,
        coloring_engine,
        color_mode: str,
        color_params: Dict
    ) -> np.ndarray:
        """Project with custom azimuth and elevation angles."""
        # Convert angles to radians
        azimuth_rad = np.radians(params.azimuth)
        elevation_rad = np.radians(params.elevation)
        
        # Create rotation matrix for viewing angle
        # First rotate around Z axis (azimuth)
        cos_az = np.cos(azimuth_rad)
        sin_az = np.sin(azimuth_rad)
        rot_z = np.array([
            [cos_az, -sin_az, 0],
            [sin_az, cos_az, 0],
            [0, 0, 1]
        ])
        
        # Then rotate around X axis (elevation)
        cos_el = np.cos(elevation_rad)
        sin_el = np.sin(elevation_rad)
        rot_x = np.array([
            [1, 0, 0],
            [0, cos_el, -sin_el],
            [0, sin_el, cos_el]
        ])
        
        # Combined rotation
        rotation = rot_x @ rot_z
        
        # Transform points
        rotated_points = points[:, :3] @ rotation.T
        
        # Project to 2D (use Y as horizontal, Z as vertical after rotation)
        x_range = params.x_range
        y_range = params.y_range
        
        width = int((x_range[1] - x_range[0]) / scale)
        height = int((y_range[1] - y_range[0]) / scale)
        
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Project points
        x_img = ((rotated_points[:, 1] - x_range[0]) / scale).astype(int)
        y_img = ((rotated_points[:, 2] - y_range[0]) / scale).astype(int)
        
        # Filter valid points
        valid = (x_img >= 0) & (x_img < width) & (y_img >= 0) & (y_img < height)
        x_img = x_img[valid]
        y_img = y_img[valid]
        points_valid = points[valid]
        depths = rotated_points[valid, 0]  # Depth along view direction
        
        if len(points_valid) == 0:
            return img
        
        # Apply coloring
        colors = coloring_engine.apply_coloring(points_valid, color_mode, color_params or {})
        
        # Draw with depth buffering
        z_buffer = np.full((height, width), np.inf)
        for x, y, z, color in zip(x_img, y_img, depths, colors):
            if z < z_buffer[height - y - 1, x]:
                z_buffer[height - y - 1, x] = z
                cv2.circle(img, (x, height - y - 1), 1, color.tolist(), -1)
        
        return img
        
    def _project_perspective(
        self,
        points: np.ndarray,
        params: ProjectionParams,
        scale: float,
        coloring_engine,
        color_mode: str,
        color_params: Dict
    ) -> np.ndarray:
        """Perspective projection with field of view."""
        # Camera position
        camera_pos = np.array([0, 0, params.camera_height])
        look_at = np.array([params.look_ahead, 0, params.camera_height])
        
        # Transform to camera coordinates
        forward = look_at - camera_pos
        forward = forward / np.linalg.norm(forward)
        right = np.cross(forward, [0, 0, 1])
        right = right / np.linalg.norm(right)
        up = np.cross(right, forward)
        
        # View matrix
        view_matrix = np.array([right, up, -forward])
        
        # Transform points to camera space
        cam_points = (points[:, :3] - camera_pos) @ view_matrix.T
        
        # Perspective projection
        fov_h_rad = np.radians(params.fov_horizontal)
        fov_v_rad = np.radians(params.fov_vertical)
        
        # Image dimensions
        width = int(100 / scale)
        height = int(60 / scale)
        
        # Project to image plane
        f_x = width / (2 * np.tan(fov_h_rad / 2))
        f_y = height / (2 * np.tan(fov_v_rad / 2))
        
        # Only project points in front of camera
        front_mask = cam_points[:, 2] < 0  # Negative Z is forward
        cam_points = cam_points[front_mask]
        points = points[front_mask]
        
        if len(points) == 0:
            return np.zeros((height, width, 3), dtype=np.uint8)
        
        # Perspective division
        x_img = (f_x * cam_points[:, 0] / -cam_points[:, 2] + width / 2).astype(int)
        y_img = (f_y * cam_points[:, 1] / -cam_points[:, 2] + height / 2).astype(int)
        
        # Create image
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Filter valid points
        valid = (x_img >= 0) & (x_img < width) & (y_img >= 0) & (y_img < height)
        x_img = x_img[valid]
        y_img = y_img[valid]
        points_valid = points[valid]
        depths = -cam_points[valid, 2]
        
        if len(points_valid) == 0:
            return img
        
        # Apply coloring
        colors = coloring_engine.apply_coloring(points_valid, color_mode, color_params or {})
        
        # Draw with depth buffering
        z_buffer = np.full((height, width), np.inf)
        for x, y, z, color in zip(x_img, y_img, depths, colors):
            if z < z_buffer[y, x]:
                z_buffer[y, x] = z
                cv2.circle(img, (x, y), 1, color.tolist(), -1)
        
        return img
        
    def _project_orthographic(
        self,
        points: np.ndarray,
        view: str,
        params: ProjectionParams,
        scale: float,
        coloring_engine,
        color_mode: str,
        color_params: Dict
    ) -> np.ndarray:
        """Standard orthographic projections (front, rear, left, right)."""
        # Map view to angles
        view_angles = {
            "front": (0, 0),
            "rear": (180, 0),
            "left": (90, 0),
            "right": (-90, 0)
        }
        
        azimuth, elevation = view_angles[view]
        params.azimuth = azimuth
        params.elevation = elevation
        
        return self._project_angled(points, params, scale, coloring_engine, color_mode, color_params)