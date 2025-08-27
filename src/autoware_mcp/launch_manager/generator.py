"""Launch file and node generation tools for AI development."""

import os
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Any
from datetime import datetime
import yaml

logger = logging.getLogger(__name__)


class LaunchGenerator:
    """Generate launch files, nodes, and configurations for Autoware."""

    def __init__(self, base_dir: Optional[Path] = None):
        """Initialize launch generator.

        Args:
            base_dir: Base directory for generated files
        """
        self.base_dir = base_dir or Path.cwd() / ".autoware-mcp"
        self.generated_dir = self.base_dir / "generated"

        # Create directory structure
        self.launches_dir = self.generated_dir / "launches"
        self.nodes_dir = self.generated_dir / "nodes"
        self.configs_dir = self.generated_dir / "configs"

        for dir_path in [self.launches_dir, self.nodes_dir, self.configs_dir]:
            dir_path.mkdir(parents=True, exist_ok=True)

    def generate_launch_file(
        self,
        name: str,
        components: List[str],
        template: Optional[str] = None,
        includes: Optional[List[str]] = None,
        remappings: Optional[Dict[str, str]] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Generate a ROS2 launch file.

        Args:
            name: Name for the launch file (without extension)
            components: List of components/nodes to include
            template: Optional template to use
            includes: Other launch files to include
            remappings: Topic remappings
            parameters: Parameters to set

        Returns:
            Path to generated launch file and metadata
        """
        # Determine version number
        version = self._get_next_version(name, self.launches_dir)
        filename = f"{name}_v{version}.launch.py"
        file_path = self.launches_dir / filename

        # Generate launch file content
        content = self._generate_launch_content(
            name, components, includes, remappings, parameters, template
        )

        # Write file
        with open(file_path, "w") as f:
            f.write(content)

        # Make executable
        os.chmod(file_path, 0o755)

        logger.info(f"Generated launch file: {file_path}")

        return {
            "success": True,
            "file_path": str(file_path),
            "filename": filename,
            "version": version,
            "components": components,
        }

    def _generate_launch_content(
        self,
        name: str,
        components: List[str],
        includes: Optional[List[str]],
        remappings: Optional[Dict[str, str]],
        parameters: Optional[Dict[str, Any]],
        template: Optional[str],
    ) -> str:
        """Generate launch file Python content."""

        if template == "perception_pipeline":
            return self._perception_pipeline_template(name, components, parameters)
        elif template == "planning_pipeline":
            return self._planning_pipeline_template(name, components, parameters)
        elif template == "control_pipeline":
            return self._control_pipeline_template(name, components, parameters)
        else:
            # Generic template
            return self._generic_template(
                name, components, includes, remappings, parameters
            )

    def _generic_template(
        self,
        name: str,
        components: List[str],
        includes: Optional[List[str]],
        remappings: Optional[Dict[str, str]],
        parameters: Optional[Dict[str, Any]],
    ) -> str:
        """Generate generic launch file template."""

        content = f'''"""Generated launch file: {name}
Generated at: {datetime.now().isoformat()}
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for {name}."""
    
    # Declare launch arguments
    declare_args = []
'''

        # Add parameter declarations
        if parameters:
            for key, value in parameters.items():
                content += f"""
    declare_args.append(
        DeclareLaunchArgument(
            '{key}',
            default_value='{value}',
            description='Parameter: {key}'
        )
    )
"""

        content += """
    # Create nodes list
    nodes = []
"""

        # Add nodes for components
        for component in components:
            content += f"""
    # {component} node
    nodes.append(
        Node(
            package='{component}_pkg',  # Update package name
            executable='{component}',
            name='{component}',
            output='screen',"""

            if remappings:
                content += """
            remappings=["""
                for old, new in remappings.items():
                    content += f"""
                ('{old}', '{new}'),"""
                content += """
            ],"""

            if parameters:
                content += """
            parameters=[{"""
                for key, value in parameters.items():
                    content += f"""
                '{key}': LaunchConfiguration('{key}'),"""
                content += """
            }],"""

            content += """
        )
    )
"""

        # Add includes
        if includes:
            content += """
    # Include other launch files
    includes = []
"""
            for include in includes:
                content += f"""
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('autoware_launch'),
                    'launch',
                    '{include}.launch.py'
                )
            ])
        )
    )
"""

        # Return launch description
        content += """
    return LaunchDescription(declare_args + nodes"""

        if includes:
            content += """ + includes"""

        content += """)
"""

        return content

    def _perception_pipeline_template(
        self, name: str, components: List[str], parameters: Optional[Dict[str, Any]]
    ) -> str:
        """Generate perception pipeline launch template."""

        content = f'''"""Perception pipeline launch file: {name}
Generated at: {datetime.now().isoformat()}
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate perception pipeline launch description."""
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Create perception container for performance
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='perception',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
'''

        # Add perception components
        if "lidar_processing" in components:
            content += """
            ComposableNode(
                package='pointcloud_preprocessor',
                plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
                name='voxel_grid_filter',
                parameters=[{
                    'voxel_size_x': 0.1,
                    'voxel_size_y': 0.1,
                    'voxel_size_z': 0.1,
                }],
                remappings=[
                    ('input', '/sensing/lidar/concatenated/pointcloud'),
                    ('output', '/perception/obstacle_segmentation/pointcloud'),
                ]
            ),
"""

        if "camera_detection" in components:
            model_path = (
                parameters.get("model_path", "./models/default.pt")
                if parameters
                else "./models/default.pt"
            )
            content += f"""
            ComposableNode(
                package='tensorrt_yolo',
                plugin='object_recognition::TensorrtYoloNodelet',
                name='camera_object_detection',
                parameters=[{{
                    'model_path': '{model_path}',
                    'score_threshold': 0.3,
                }}],
                remappings=[
                    ('in/image', '/sensing/camera/image_rect_color'),
                    ('out/objects', '/perception/object_recognition/objects'),
                ]
            ),
"""

        if "fusion" in components:
            content += """
            ComposableNode(
                package='multi_object_tracker',
                plugin='MultiObjectTracker',
                name='multi_object_tracker',
                parameters=[{
                    'world_frame_id': 'map',
                    'publish_rate': 10.0,
                }],
                remappings=[
                    ('input/objects', '/perception/object_recognition/objects'),
                    ('output/objects', '/perception/object_recognition/tracking/objects'),
                ]
            ),
"""

        content += """
        ],
        output='screen',
    )
    
    return LaunchDescription([
        use_sim_time,
        perception_container,
    ])
"""

        return content

    def _planning_pipeline_template(
        self, name: str, components: List[str], parameters: Optional[Dict[str, Any]]
    ) -> str:
        """Generate planning pipeline launch template."""

        # Similar structure for planning components
        # This would include behavior planner, motion planner, etc.
        return self._generic_template(name, components, None, None, parameters)

    def _control_pipeline_template(
        self, name: str, components: List[str], parameters: Optional[Dict[str, Any]]
    ) -> str:
        """Generate control pipeline launch template."""

        # Similar structure for control components
        # This would include trajectory follower, vehicle cmd gate, etc.
        return self._generic_template(name, components, None, None, parameters)

    def generate_node_config(
        self, node_name: str, parameters: Dict[str, Any], format: str = "yaml"
    ) -> Dict[str, Any]:
        """Generate node configuration file.

        Args:
            node_name: Name of the node
            parameters: Parameters to configure
            format: Config format (yaml or json)

        Returns:
            Path to generated config file
        """
        version = self._get_next_version(node_name, self.configs_dir)

        if format == "yaml":
            filename = f"{node_name}_v{version}.param.yaml"
            file_path = self.configs_dir / filename

            # Generate YAML content
            import yaml

            content = {f"/{node_name}": {"ros__parameters": parameters}}

            with open(file_path, "w") as f:
                yaml.dump(content, f, default_flow_style=False)
        else:
            filename = f"{node_name}_v{version}.json"
            file_path = self.configs_dir / filename

            # Generate JSON content with same structure as YAML
            content = {f"/{node_name}": {"ros__parameters": parameters}}

            with open(file_path, "w") as f:
                json.dump(content, f, indent=2)

        logger.info(f"Generated config file: {file_path}")

        return {
            "success": True,
            "file_path": str(file_path),
            "filename": filename,
            "version": version,
            "format": format,
        }

    def generate_custom_node(
        self,
        name: str,
        language: str = "python",
        node_type: str = "basic",
        interfaces: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Generate a custom ROS2 node.

        Args:
            name: Name of the node
            language: Programming language (python or cpp)
            node_type: Type of node (basic, service, action)
            interfaces: List of topics/services to interface with

        Returns:
            Path to generated node file
        """
        version = self._get_next_version(name, self.nodes_dir)

        if language == "python":
            filename = f"{name}_v{version}.py"
            file_path = self.nodes_dir / filename
            content = self._generate_python_node(name, node_type, interfaces)

            with open(file_path, "w") as f:
                f.write(content)

            # Make executable
            os.chmod(file_path, 0o755)
        else:
            filename = f"{name}_v{version}.cpp"
            file_path = self.nodes_dir / filename
            content = self._generate_cpp_node(name, node_type, interfaces)

            with open(file_path, "w") as f:
                f.write(content)

        logger.info(f"Generated custom node: {file_path}")

        return {
            "success": True,
            "file_path": str(file_path),
            "filename": filename,
            "version": version,
            "language": language,
            "node_type": node_type,
        }

    def _generate_python_node(
        self, name: str, node_type: str, interfaces: Optional[List[str]]
    ) -> str:
        """Generate Python node template."""

        content = f'''#!/usr/bin/env python3
"""Generated ROS2 node: {name}
Generated at: {datetime.now().isoformat()}
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
'''

        if node_type == "service":
            content += """from example_interfaces.srv import AddTwoInts
"""

        content += f'''

class {name.title().replace("_", "")}Node(Node):
    """Custom ROS2 node: {name}."""
    
    def __init__(self):
        super().__init__('{name}')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        
'''

        if interfaces:
            for interface in interfaces:
                content += f"""        # Interface: {interface}
        self.publisher_ = self.create_publisher(String, '{interface}', 10)
"""

        if node_type == "service":
            content += '''
        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.service_callback
        )
        
    def service_callback(self, request, response):
        """Handle service request."""
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response
'''
        else:
            content += '''
        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.timer_callback
        )
        
        self.counter = 0
        
    def timer_callback(self):
        """Timer callback for periodic processing."""
        msg = String()
        msg.data = f'Message {self.counter}'
        
'''
            if interfaces:
                content += """        self.publisher_.publish(msg)
"""

            content += """        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1
"""

        content += f'''

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = {name.title().replace("_", "")}Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

        return content

    def _generate_cpp_node(
        self, name: str, node_type: str, interfaces: Optional[List[str]]
    ) -> str:
        """Generate C++ node template."""

        # Basic C++ node template
        return f'''// Generated ROS2 node: {name}
// Generated at: {datetime.now().isoformat()}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class {name.title().replace("_", "")}Node : public rclcpp::Node
{{
public:
    {name.title().replace("_", "")}Node() : Node("{name}")
    {{
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("output", 10);
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&{name.title().replace("_", "")}Node::timer_callback, this)
        );
    }}

private:
    void timer_callback()
    {{
        auto message = std_msgs::msg::String();
        message.data = "Hello from {name}";
        publisher_->publish(message);
    }}
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
}};

int main(int argc, char * argv[])
{{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{name.title().replace("_", "")}Node>());
    rclcpp::shutdown();
    return 0;
}}
'''

    def _get_next_version(self, name: str, directory: Path) -> int:
        """Get next version number for a file."""
        existing = list(directory.glob(f"{name}_v*.*"))
        if not existing:
            return 1

        versions = []
        for file in existing:
            try:
                # Extract version number from filename like test_v1.launch.py
                # First get the base name without all extensions
                base_name = file.name
                # Remove everything after _v and before first dot
                if "_v" in base_name:
                    version_part = base_name.split("_v")[1].split(".")[0]
                    versions.append(int(version_part))
            except (ValueError, IndexError):
                continue

        return max(versions) + 1 if versions else 1

    def validate_launch_file(self, file_path: str) -> Dict[str, Any]:
        """Validate a launch file syntax.

        Args:
            file_path: Path to launch file

        Returns:
            Validation results
        """
        path = Path(file_path)

        if not path.exists():
            return {"success": False, "error": f"File not found: {file_path}"}

        # Check if it's a Python file
        if not path.suffix == ".py":
            return {"success": False, "error": "Launch file must be a Python file"}

        # Try to compile the Python file
        try:
            with open(path) as f:
                code = f.read()

            compile(code, file_path, "exec")

            # Check for required function
            if "generate_launch_description" not in code:
                return {
                    "success": False,
                    "error": "Missing required function: generate_launch_description",
                }

            return {"success": True, "message": "Launch file is valid"}

        except SyntaxError as e:
            return {"success": False, "error": f"Syntax error: {e}"}
        except Exception as e:
            return {"success": False, "error": f"Validation error: {e}"}

    def list_generated_files(self) -> Dict[str, List[str]]:
        """List all generated files.

        Returns:
            Dictionary of generated files by category
        """
        files = {"launches": [], "nodes": [], "configs": []}

        for launch_file in self.launches_dir.glob("*"):
            if launch_file.is_file():
                files["launches"].append(str(launch_file))

        for node_file in self.nodes_dir.glob("*"):
            if node_file.is_file():
                files["nodes"].append(str(node_file))

        for config_file in self.configs_dir.glob("*"):
            if config_file.is_file():
                files["configs"].append(str(config_file))

        return files
