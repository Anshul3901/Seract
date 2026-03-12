from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # Add launch argument
    zero_pose_arg = DeclareLaunchArgument(
        'zero_pose',
        default_value='false',
        description='Test zero pose after startup'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Visualize the robot in RViz'
    )

    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='5.0',
        description='Seconds to wait before spawning controllers to allow controller_manager to initialize'
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('so_100_arm'), 'config', 'so_100_arm.urdf.xacro']
                ),
                ' ',
                'use_fake_hardware:=false'
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # controller_manager should obtain the robot description from the '~/robot_description' topic
    # published by robot_state_publisher. Do NOT pass robot_description directly as a parameter.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('so_100_arm'), 'config', 'ros2_controllers.yaml']
            ),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Start spawners with a short delay so the controller_manager has time to finish hardware activation
    joint_state_broadcaster_timer = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[joint_state_broadcaster_spawner]
    )

    # Delay rviz start after joint_state_broadcaster
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so_100_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so_100_arm_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Spawn the gripper controller with a delay
    gripper_controller_timer = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[gripper_controller_spawner]
    )

    # Delay loading and starting robot_controller after joint_state_broadcaster
    # This ensures proper sequencing: joint_state_broadcaster -> robot_controller
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('so_100_arm'), 'config', 'urdf.rviz'])]
    )

    # Add zero pose test node
    zero_pose_node = Node(
        condition=IfCondition(LaunchConfiguration('zero_pose')),
        package='so_arm_100_hardware',
        executable='zero_pose.py',
        name='zero_pose_test',
    )

    nodes = [
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_timer,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gripper_controller_timer,
        rviz_node,
        zero_pose_node
    ]

    return LaunchDescription([zero_pose_arg, rviz_arg, spawn_delay_arg] + nodes) 
