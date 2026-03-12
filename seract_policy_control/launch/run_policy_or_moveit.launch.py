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
        actions.append(Node(
            package='seract_policy_control',
            executable='lerobot_policy_controller',
            name='lerobot_policy_controller',
            output='screen',
            parameters=[cfg, {'use_open_loop_test': use_ol}],
            env={
                'RMW_FASTRTPS_USE_SHM': '0',
                'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', ''),
                'PYTHONPATH': python_path + ':' + os.environ.get('PYTHONPATH', ''),
                'HOME': os.environ.get('HOME', os.path.expanduser('~'))
            }
        ))

    # NOTE: We don't spawn move_group here on purpose—you usually want it in its own terminal.
    # Start it separately with:
    #   ros2 launch so101_moveit_config move_group.launch.py
    return actions

