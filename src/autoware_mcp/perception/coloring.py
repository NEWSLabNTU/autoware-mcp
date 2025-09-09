"""
Point cloud coloring engine for various visualization modes.
"""

import numpy as np
from typing import Dict, Any, Tuple
import cv2


class ColoringEngine:
    """Handles point cloud coloring with multiple modes and customization."""
    
    def __init__(self):
        """Initialize coloring engine with default palettes."""
        # Define color palettes
        self.palettes = {
            "viridis": self._generate_colormap("viridis"),
            "jet": self._generate_colormap("jet"),
            "rainbow": self._generate_colormap("rainbow"),
            "coolwarm": self._generate_colormap("coolwarm"),
            "grayscale": self._generate_colormap("gray"),
        }
        
        # Default parameters for each mode
        self.default_params = {
            "height": {
                "min_height": -2.0,
                "max_height": 10.0,
                "palette": "viridis"
            },
            "distance": {
                "min_distance": 0.0,
                "max_distance": 50.0,
                "palette": "jet"
            },
            "radial_distance": {
                "min_distance": 0.0,
                "max_distance": 30.0,
                "palette": "coolwarm"
            },
            "intensity": {
                "min_intensity": 0.0,
                "max_intensity": 255.0,
                "palette": "grayscale"
            },
            "azimuth": {
                "palette": "rainbow"
            },
            "uniform": {
                "color": [0, 255, 0]  # Green
            },
            "semantic": {
                "class_colors": {
                    0: [128, 128, 128],  # Unknown - gray
                    1: [255, 0, 0],      # Vehicle - red
                    2: [0, 255, 0],      # Pedestrian - green
                    3: [0, 0, 255],      # Cyclist - blue
                    4: [255, 255, 0],    # Road - yellow
                    5: [255, 0, 255],    # Sidewalk - magenta
                    6: [0, 255, 255],    # Building - cyan
                    7: [128, 0, 0],      # Vegetation - dark red
                    8: [0, 128, 0],      # Traffic sign - dark green
                }
            }
        }
        
    def apply_coloring(
        self,
        points: np.ndarray,
        mode: str,
        params: Dict[str, Any]
    ) -> np.ndarray:
        """
        Apply coloring to point cloud based on mode.
        
        Args:
            points: Point cloud array (Nx3 or Nx4 with XYZI)
            mode: Coloring mode
            params: Mode-specific parameters
            
        Returns:
            Colors array (Nx3) with BGR values
        """
        # Merge with default parameters
        mode_params = self.default_params.get(mode, {}).copy()
        mode_params.update(params)
        
        # Route to appropriate coloring method
        if mode == "height":
            return self._color_by_height(points, mode_params)
        elif mode == "distance":
            return self._color_by_distance(points, mode_params)
        elif mode == "radial_distance":
            return self._color_by_radial_distance(points, mode_params)
        elif mode == "intensity":
            return self._color_by_intensity(points, mode_params)
        elif mode == "azimuth":
            return self._color_by_azimuth(points, mode_params)
        elif mode == "uniform":
            return self._color_uniform(points, mode_params)
        elif mode == "semantic":
            return self._color_by_semantic(points, mode_params)
        elif mode == "velocity":
            return self._color_by_velocity(points, mode_params)
        elif mode == "density":
            return self._color_by_density(points, mode_params)
        else:
            # Default to height coloring
            return self._color_by_height(points, mode_params)
            
    def _color_by_height(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by Z coordinate (height)."""
        if len(points) == 0:
            return np.array([])
            
        z_values = points[:, 2]
        min_z = params.get("min_height", z_values.min())
        max_z = params.get("max_height", z_values.max())
        
        # Normalize to 0-1
        if max_z > min_z:
            normalized = (z_values - min_z) / (max_z - min_z)
        else:
            normalized = np.zeros_like(z_values)
            
        # Apply colormap
        palette_name = params.get("palette", "viridis")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _color_by_distance(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by Euclidean distance from origin."""
        if len(points) == 0:
            return np.array([])
            
        distances = np.linalg.norm(points[:, :3], axis=1)
        min_d = params.get("min_distance", 0.0)
        max_d = params.get("max_distance", distances.max())
        
        # Normalize
        if max_d > min_d:
            normalized = (distances - min_d) / (max_d - min_d)
            normalized = np.clip(normalized, 0, 1)
        else:
            normalized = np.zeros_like(distances)
            
        # Apply colormap
        palette_name = params.get("palette", "jet")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _color_by_radial_distance(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by XY plane distance (radial distance)."""
        if len(points) == 0:
            return np.array([])
            
        radial_distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        min_d = params.get("min_distance", 0.0)
        max_d = params.get("max_distance", radial_distances.max())
        
        # Normalize
        if max_d > min_d:
            normalized = (radial_distances - min_d) / (max_d - min_d)
            normalized = np.clip(normalized, 0, 1)
        else:
            normalized = np.zeros_like(radial_distances)
            
        # Apply colormap
        palette_name = params.get("palette", "coolwarm")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _color_by_intensity(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by intensity (4th channel if available)."""
        if len(points) == 0:
            return np.array([])
            
        # Check if intensity is available
        if points.shape[1] >= 4:
            intensities = points[:, 3]
        else:
            # Use distance as fallback
            return self._color_by_distance(points, params)
            
        min_i = params.get("min_intensity", intensities.min())
        max_i = params.get("max_intensity", intensities.max())
        
        # Normalize
        if max_i > min_i:
            normalized = (intensities - min_i) / (max_i - min_i)
            normalized = np.clip(normalized, 0, 1)
        else:
            normalized = np.zeros_like(intensities)
            
        # Apply colormap
        palette_name = params.get("palette", "grayscale")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _color_by_azimuth(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by azimuth angle (angle in XY plane)."""
        if len(points) == 0:
            return np.array([])
            
        # Calculate azimuth angle (-pi to pi)
        azimuths = np.arctan2(points[:, 1], points[:, 0])
        
        # Normalize to 0-1
        normalized = (azimuths + np.pi) / (2 * np.pi)
        
        # Apply rainbow colormap for angle visualization
        palette_name = params.get("palette", "rainbow")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _color_uniform(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Apply uniform color to all points."""
        if len(points) == 0:
            return np.array([])
            
        color = params.get("color", [0, 255, 0])  # Default green
        colors = np.tile(color, (len(points), 1)).astype(np.uint8)
        
        return colors
        
    def _color_by_semantic(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by semantic class (requires class labels)."""
        if len(points) == 0:
            return np.array([])
            
        # Check if semantic labels are provided
        if "labels" not in params:
            # Fallback to uniform coloring
            return self._color_uniform(points, {"color": [128, 128, 128]})
            
        labels = params["labels"]
        class_colors = params.get("class_colors", self.default_params["semantic"]["class_colors"])
        
        # Map labels to colors
        colors = np.zeros((len(points), 3), dtype=np.uint8)
        for i, label in enumerate(labels):
            if label in class_colors:
                colors[i] = class_colors[label]
            else:
                colors[i] = [128, 128, 128]  # Default gray for unknown
                
        return colors
        
    def _color_by_velocity(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by velocity (requires velocity data)."""
        if len(points) == 0:
            return np.array([])
            
        # Check if velocities are provided
        if "velocities" not in params:
            # Fallback to distance coloring
            return self._color_by_distance(points, params)
            
        velocities = params["velocities"]
        speeds = np.linalg.norm(velocities, axis=1)
        
        min_v = params.get("min_velocity", 0.0)
        max_v = params.get("max_velocity", 10.0)
        
        # Normalize
        if max_v > min_v:
            normalized = (speeds - min_v) / (max_v - min_v)
            normalized = np.clip(normalized, 0, 1)
        else:
            normalized = np.zeros_like(speeds)
            
        # Apply colormap
        palette_name = params.get("palette", "jet")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _color_by_density(self, points: np.ndarray, params: Dict) -> np.ndarray:
        """Color points by local density."""
        if len(points) == 0:
            return np.array([])
            
        # Calculate local density using KD-tree
        from scipy.spatial import KDTree
        
        radius = params.get("radius", 1.0)
        tree = KDTree(points[:, :3])
        
        # Count neighbors within radius
        densities = np.zeros(len(points))
        for i, point in enumerate(points[:, :3]):
            neighbors = tree.query_ball_point(point, radius)
            densities[i] = len(neighbors)
            
        # Normalize
        min_d = densities.min()
        max_d = densities.max()
        
        if max_d > min_d:
            normalized = (densities - min_d) / (max_d - min_d)
        else:
            normalized = np.zeros_like(densities)
            
        # Apply colormap
        palette_name = params.get("palette", "viridis")
        colors = self._apply_colormap(normalized, palette_name)
        
        return colors
        
    def _apply_colormap(self, values: np.ndarray, palette_name: str) -> np.ndarray:
        """Apply a colormap to normalized values."""
        if palette_name in self.palettes:
            colormap = self.palettes[palette_name]
        else:
            # Default to viridis
            colormap = self.palettes["viridis"]
            
        # Map normalized values to colors
        indices = (values * 255).astype(np.uint8)
        colors = colormap[indices]
        
        return colors
        
    def _generate_colormap(self, name: str) -> np.ndarray:
        """Generate a 256-color colormap."""
        # Use OpenCV colormaps
        colormap_ids = {
            "viridis": cv2.COLORMAP_VIRIDIS,
            "jet": cv2.COLORMAP_JET,
            "rainbow": cv2.COLORMAP_RAINBOW,
            "coolwarm": cv2.COLORMAP_HOT,  # Use HOT as alternative to COOLWARM
            "gray": cv2.COLORMAP_BONE,
        }
        
        if name in colormap_ids:
            # Generate colormap
            colormap = np.zeros((256, 1, 3), dtype=np.uint8)
            for i in range(256):
                colormap[i, 0, :] = [i, i, i]
                
            colormap = cv2.applyColorMap(colormap, colormap_ids[name])
            colormap = colormap.reshape(256, 3)
            
            # Convert from RGB to BGR for OpenCV
            colormap = colormap[:, ::-1]
        else:
            # Default grayscale
            colormap = np.stack([np.arange(256)] * 3, axis=1).astype(np.uint8)
            
        return colormap
        
    def create_legend(
        self,
        mode: str,
        params: Dict,
        width: int = 200,
        height: int = 30
    ) -> np.ndarray:
        """
        Create a color legend for the specified mode.
        
        Args:
            mode: Coloring mode
            params: Mode parameters
            width: Legend width
            height: Legend height
            
        Returns:
            Legend image as numpy array
        """
        # Create gradient
        gradient = np.linspace(0, 1, width).reshape(1, -1)
        gradient = np.repeat(gradient, height, axis=0)
        
        # Apply colormap
        mode_params = self.default_params.get(mode, {}).copy()
        mode_params.update(params)
        palette_name = mode_params.get("palette", "viridis")
        
        # Get colors
        colors = self._apply_colormap(gradient[0], palette_name)
        legend_img = np.zeros((height, width, 3), dtype=np.uint8)
        
        for i in range(width):
            legend_img[:, i] = colors[i]
            
        # Add labels
        if mode in ["height", "distance", "radial_distance", "intensity", "velocity"]:
            # Add min/max labels
            min_val = mode_params.get(f"min_{mode}", 0)
            max_val = mode_params.get(f"max_{mode}", 100)
            
            # Draw text
            cv2.putText(legend_img, f"{min_val:.1f}", (5, height - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(legend_img, f"{max_val:.1f}", (width - 40, height - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                       
        return legend_img