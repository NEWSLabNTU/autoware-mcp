#!/usr/bin/env python3

"""
Launch file for Autoware planning simulation via MCP.
This wraps the autoware_launch planning_simulator.launch.xml
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    autoware_launch_dir = get_package_share_directory('autoware_launch')
    
    # Default paths
    default_map_path = os.path.expanduser("~/autoware_map/sample-map-planning")
    default_vehicle_model = "sample_vehicle"
    default_sensor_model = "sample_sensor_kit"
    
    # Include the main planning simulator launch file
    planning_simulator = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([autoware_launch_dir, 'launch', 'planning_simulator.launch.xml'])
        ]),
        launch_arguments={
            'map_path': os.environ.get('MAP_PATH', default_map_path),
            'vehicle_model': os.environ.get('VEHICLE_MODEL', default_vehicle_model),
            'sensor_model': os.environ.get('SENSOR_MODEL', default_sensor_model),
        }.items()
    )
    
    return LaunchDescription([
        planning_simulator
    ])