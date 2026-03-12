from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Keep the current shell's ROS env; just scrub conda + set SHM off
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('CONDA_PREFIX', ''),
        SetEnvironmentVariable('CONDA_DEFAULT_ENV', ''),
        SetEnvironmentVariable('CONDA_EXE', ''),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        # DO NOT override LD_LIBRARY_PATH or PATH here; let the sourced ROS env provide them.

        DeclareLaunchArgument('controller_topic', default_value='/so_100_arm_controller/joint_trajectory'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('command_dt', default_value='0.25'),
        DeclareLaunchArgument('jog_joint_index', default_value='0'),
        DeclareLaunchArgument('jog_joint_indices', default_value=''),
        DeclareLaunchArgument('jog_amplitude', default_value='0.06'),
        DeclareLaunchArgument('jog_frequency', default_value='0.2'),
        DeclareLaunchArgument('max_step_rad', default_value='0.05'),

        OpaqueFunction(function=launch_node),
    ])

def launch_node(context):
    import ast
    
    # Get parameter values
    jog_joint_indices_str = LaunchConfiguration('jog_joint_indices').perform(context)
    
    # Parse jog_joint_indices if provided, otherwise use empty list
    jog_joint_indices = []
    if jog_joint_indices_str and jog_joint_indices_str.strip():
        try:
            # Try to parse as Python list literal
            jog_joint_indices = ast.literal_eval(jog_joint_indices_str)
            if not isinstance(jog_joint_indices, list):
                jog_joint_indices = []
        except (ValueError, SyntaxError):
            # If parsing fails, use empty list
            jog_joint_indices = []
    
    # Build parameters dict
    params = {
        'controller_topic': LaunchConfiguration('controller_topic'),
        'rate_hz': LaunchConfiguration('rate_hz'),
        'command_dt': LaunchConfiguration('command_dt'),
        'jog_joint_index': LaunchConfiguration('jog_joint_index'),
        'jog_amplitude': LaunchConfiguration('jog_amplitude'),
        'jog_frequency': LaunchConfiguration('jog_frequency'),
        'max_step_rad': LaunchConfiguration('max_step_rad'),
    }
    
    # Only add jog_joint_indices if it's not empty
    if jog_joint_indices:
        params['jog_joint_indices'] = jog_joint_indices
    
    return [
        Node(
            package='seract_policy_control',
            executable='joint_jogger',
            name='joint_jogger',
            output='screen',
            parameters=[params],
        ),
    ]
