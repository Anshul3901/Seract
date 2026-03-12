from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    args = [
        DeclareLaunchArgument('policy_url', default_value='http://127.0.0.1:8009/plan_joints'),
        DeclareLaunchArgument('controller_topic', default_value='/so_100_arm_controller/joint_trajectory'),
        DeclareLaunchArgument('command_dt', default_value='0.7'),
        DeclareLaunchArgument('wrist_camera_device', default_value='/dev/video7'),
        DeclareLaunchArgument('use_fake_policy', default_value='false'),
        DeclareLaunchArgument('enable_topic', default_value='/policy_bridge/enable'),
        DeclareLaunchArgument('start_enabled', default_value='false'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch)])

def _launch(context):
    use_fake_policy = LaunchConfiguration('use_fake_policy').perform(context).lower() == 'true'

    return [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
        ),

        Node(
            package='seract_policy_control',
            executable='policy_bridge',
            name='policy_bridge',
            output='screen',
            parameters=[{
                'enable_topic': LaunchConfiguration('enable_topic'),
                'start_enabled': ParameterValue(LaunchConfiguration('start_enabled'), value_type=bool),

                'policy_url':       LaunchConfiguration('policy_url'),
                'controller_topic': LaunchConfiguration('controller_topic'),
                'command_dt':       LaunchConfiguration('command_dt'),

                'ros_joint_names': [
                    'Shoulder_Rotation',
                    'Shoulder_Pitch',
                    'Elbow',
                    'Wrist_Pitch',
                    'Wrist_Roll'
                ],
                'joint_min': [-3.141592653589793, -1.57, -2.50, -2.50, -2.80],
                'joint_max': [ 3.141592653589793,  1.57,  2.50,  2.50,  2.80],
                'max_step_rad': 0.15,
                'delta_scale': 1.0,
                'force_absolute_mode': True,
                'startup_hold_ticks': 2,
                'startup_ramp_seconds': 3.0,
                'startup_warmup_max_step_rad': 0.05,
                'use_fake_policy': use_fake_policy,
            }],
        ),
    ]
