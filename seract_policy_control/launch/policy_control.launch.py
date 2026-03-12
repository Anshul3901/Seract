from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('seract_policy_control')
    cfg = os.path.join(pkg, 'config', 'policy_control.yaml')
    node = Node(
        package='seract_policy_control',
        executable='lerobot_policy_controller',
        output='screen',
        parameters=[cfg]
    )
    return LaunchDescription([node])

