from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    args = [
        DeclareLaunchArgument('run_policy',  default_value='true'),
        DeclareLaunchArgument('run_moveit',  default_value='false'),
        DeclareLaunchArgument('run_tf',      default_value='true'),
        DeclareLaunchArgument('use_open_loop_test', default_value='false'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch)])

def _launch(context):
    run_policy  = LaunchConfiguration('run_policy').perform(context)  == 'true'
    run_moveit  = LaunchConfiguration('run_moveit').perform(context)  == 'true'
    run_tf      = LaunchConfiguration('run_tf').perform(context)      == 'true'
    use_ol      = LaunchConfiguration('use_open_loop_test').perform(context) == 'true'



    

    actions = []

    # Always start USB camera node for wrist camera
    actions.append(Node(
        package='seract_pipeline',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',  # USB webcam device
            'image_topic': '/camera2/color/image_raw',  # Wrist camera topic
            'frame_id': 'camera2_link',
            'image_width': 640,
            'image_height': 480,
            'fps': 30.0
        }],
        env={
            'RMW_FASTRTPS_USE_SHM': '0',
            'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', ''),
            'HOME': os.environ.get('HOME', os.path.expanduser('~'))
        }
    ))

    if run_tf:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_base',
            arguments=['0.40','0.00','0.60','0','0','0','1','base_link','camera_link'],
            output='screen',
            env={
                'RMW_FASTRTPS_USE_SHM': '0',
                'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', ''),
                'HOME': os.environ.get('HOME', os.path.expanduser('~'))
            }
        ))

    if run_policy:
        pkg = get_package_share_directory('seract_policy_control')
        cfg = os.path.join(pkg, 'config', 'policy_control.yaml')
        # Get install directory for Python path
        install_dir = os.path.join(os.path.expanduser('~'), 'seract_ws', 'install')
        python_path = os.path.join(install_dir, 'seract_policy_control', 'lib', 'python3.10', 'site-packages')
        # Ensure NumPy 1.x is used (for cv_bridge compatibility)
        # Build PYTHONPATH that prioritizes system packages over conda
        system_site_packages = '/usr/lib/python3.10/dist-packages'
        # Start with system packages first, then workspace, then conda (if present)
        python_path_parts = [system_site_packages, python_path]
        if 'PYTHONPATH' in os.environ:
            # Filter out conda paths from existing PYTHONPATH
            existing_paths = os.environ.get('PYTHONPATH', '').split(':')
            for p in existing_paths:
                if p and 'conda' not in p.lower() and p not in python_path_parts:
                    python_path_parts.append(p)
        
        env_dict = {
            'RMW_FASTRTPS_USE_SHM': '0',
            'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', ''),
            'PYTHONPATH': ':'.join(python_path_parts),
            'HOME': os.environ.get('HOME', os.path.expanduser('~'))
        }
        # Remove conda from PATH if present to avoid NumPy conflicts
        if 'CONDA_PREFIX' in os.environ:
            # Filter conda paths from PATH
            path_parts = os.environ.get('PATH', '').split(':')
            filtered_path = [p for p in path_parts if 'conda' not in p.lower()]
            env_dict['PATH'] = ':'.join(filtered_path)
        actions.append(Node(
            package='seract_policy_control',
            executable='lerobot_policy_controller',
            name='lerobot_policy_controller',
            output='screen',
            parameters=[cfg, {'use_open_loop_test': use_ol}],
            env=env_dict
        ))

    # NOTE: We don't spawn move_group here on purpose—you usually want it in its own terminal.
    # Start it separately with:
    #   ros2 launch so101_moveit_config move_group.launch.py
    return actions

