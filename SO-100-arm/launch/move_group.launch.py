from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
import os

def generate_launch_description():
    # Build the MoveIt configuration for your so_100_arm
    moveit_config = (
        MoveItConfigsBuilder("so_100_arm", package_name="so_100_arm")
        .robot_description(file_path="config/so_100_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/so_100_arm.srdf")
        .trajectory_execution(file_path="config/controllers_moveit.yaml")  # controller mapping
        .to_moveit_configs()
    )

    # Generate standard MoveIt move_group launch
    return generate_move_group_launch(moveit_config)
