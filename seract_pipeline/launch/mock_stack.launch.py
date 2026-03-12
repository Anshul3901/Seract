#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    package_share_dir = get_package_share_directory('seract_pipeline')
    
    # Path to config file
    config_file = os.path.join(package_share_dir, 'config', 'pipeline.yaml')
    
    # Declare launch arguments
    use_static_tf_arg = DeclareLaunchArgument(
        'use_static_tf',
        default_value='true',
        description='Whether to publish a static transform from base_link to camera_link'
    )
    
    # Mock primitive server node
    mock_primitive_server_node = Node(
        package='seract_primitives',
        executable='mock_primitive_server',
        name='mock_primitive_server',
        output='screen'
    )
    
    # Pipeline node
    pipeline_node = Node(
        package='seract_pipeline',
        executable='pipeline',
        name='pipeline',
        parameters=[config_file],
        output='screen'
    )
    
    # Static transform publisher (optional)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.4', '0.0', '0.6',  # x, y, z translation
            '0.0', '0.0', '0.0', '1.0',  # quaternion (identity)
            'base_link',
            'camera_link'
        ],
        condition=IfCondition(LaunchConfiguration('use_static_tf')),
        output='screen'
    )
    
    return LaunchDescription([
        use_static_tf_arg,
        mock_primitive_server_node,
        pipeline_node,
        static_tf_node,
    ])

