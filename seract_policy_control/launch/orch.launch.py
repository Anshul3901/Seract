from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_delay_sec',
            default_value='3.0',
            description='Seconds to wait before running pick & place (hardware warmup)'
        ),
        OpaqueFunction(function=_launch)
    ])


def _launch(context):
    start_delay_sec = float(LaunchConfiguration('start_delay_sec').perform(context))

    # 1) Hardware
    hardware = ExecuteProcess(
        cmd=['ros2', 'launch', 'so_100_arm', 'hardware.launch.py'],
        output='screen'
    )

    # 2) Policy bridge launch (START DISABLED so it doesn't override pick_place)
    policy_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'seract_policy_control', 'policy_bridge.launch.py',
            'start_enabled:=false'
        ],
        output='screen'
    )

    # 3) Pick & place (one-shot)
    pick_place = ExecuteProcess(
        cmd=['ros2', 'run', 'so101_motion', 'pick_place'],
        output='screen'
    )

    # After pick_place exits, enable policy bridge
    enable_policy = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/policy_bridge/enable',
            'std_msgs/msg/Bool',
            '{data: true}'
        ],
        output='screen'
    )

    return [
        hardware,

        # Start policy bridge right away, but disabled
        policy_bridge,

        # Wait for hardware to come up before motion
        TimerAction(
            period=start_delay_sec,
            actions=[pick_place]
        ),

        # When pick_place finishes, enable policy_bridge
        RegisterEventHandler(
            OnProcessExit(
                target_action=pick_place,
                on_exit=[enable_policy],
            )
        ),
    ]
