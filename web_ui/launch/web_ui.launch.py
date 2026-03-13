from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Get workspace path
    workspace_path = os.path.expanduser('~/seract_orch_ws')
    
    return LaunchDescription([
        # ROS2 Bridge Node
        Node(
            package='web_ui',
            executable='web_ui_bridge',
            name='web_ui_bridge',
            output='screen',
        ),
        
        # Flask Web Server (run in background)
        ExecuteProcess(
            cmd=['python3', '-m', 'web_ui.web_ui_server'],
            cwd=os.path.join(workspace_path, 'src', 'web_ui'),
            output='screen',
        ),
    ])

