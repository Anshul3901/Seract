from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg = get_package_share_directory('so_100_arm')

    xacro_file = os.path.join(pkg, 'config', 'so_100_arm.urdf.xacro')
    srdf_file  = os.path.join(pkg, 'config', 'so_100_arm.srdf')
    rviz_cfg   = os.path.join(pkg, 'config', 'moveit.rviz')
    kinematics = os.path.join(pkg, 'config', 'kinematics.yaml')
    controllers = os.path.join(pkg, 'config', 'moveit_controllers.yaml')

    print(f"Loading controllers from: {controllers}")  # ✅ helps verify during launch

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                controllers,  # ✅ include controllers here
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                controllers,  # ✅ and here
            ],
        ),
    ])
