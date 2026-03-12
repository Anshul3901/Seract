#!/usr/bin/env python
"""
LeRobot configuration for SO101 robot via ROS2.
This config matches the actual hardware setup with Feetech motors.
"""

from dataclasses import dataclass, field
from enum import Enum
from lerobot.robots.config import RobotConfig

# Define ActionType enum if not already available
class ActionType(Enum):
    JOINT_POSITION = "joint_position"
    JOINT_VELOCITY = "joint_velocity"
    JOINT_TORQUE = "joint_torque"
    CARTESIAN = "cartesian"

# Define ROS2InterfaceConfig if not already available
@dataclass
class ROS2InterfaceConfig:
    base_link: str = "base_link"
    arm_joint_names: list[str] = field(default_factory=list)
    gripper_joint_name: str | None = None
    gripper_open_position: float = 0.0
    gripper_close_position: float = 1.0
    max_linear_velocity: float = 0.05  # m/s
    max_angular_velocity: float = 0.25  # rad/s
    controller_topic: str = "/so_100_arm_controller/joint_trajectory"
    joint_state_topic: str = "/joint_states"

# Define ROS2Config base class if not already available
@dataclass
class ROS2Config(RobotConfig):
    action_type: ActionType = ActionType.JOINT_POSITION
    ros2_interface: ROS2InterfaceConfig = field(default_factory=ROS2InterfaceConfig)

@RobotConfig.register_subclass("so101_ros2")
@dataclass
class SO101ROS2Config(ROS2Config):
    """
    LeRobot configuration for SO101 robot via ROS2.
    
    This configuration connects to the SO101 robot through ROS2 topics:
    - Joint states: /joint_states
    - Controller: /so_100_arm_controller/joint_trajectory
    
    Joint names match the actual hardware:
    - Arm: Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll
    - Gripper: Gripper
    """
    
    action_type: ActionType = ActionType.JOINT_POSITION

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=[
                "Shoulder_Rotation",
                "Shoulder_Pitch",
                "Elbow",
                "Wrist_Pitch",
                "Wrist_Roll",
            ],
            gripper_joint_name="Gripper",
            gripper_open_position=0.0,
            gripper_close_position=1.0,
            max_linear_velocity=0.05,  # m/s
            max_angular_velocity=0.25,  # rad/s
            controller_topic="/so_100_arm_controller/joint_trajectory",
            joint_state_topic="/joint_states",
        )
    )

