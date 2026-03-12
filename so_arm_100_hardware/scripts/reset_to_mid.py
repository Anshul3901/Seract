#!/usr/bin/env python3
"""
Publish mid position directly to controller topics.
Simple script that publishes positions and exits.
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import time

# Initialize ROS
rclpy.init()
node = Node('reset_to_mid')

# Create publishers
arm_pub = node.create_publisher(JointTrajectory, '/so_100_arm_controller/joint_trajectory', 10)
gripper_pub = node.create_publisher(Float64MultiArray, '/so_100_arm_gripper_controller/commands', 10)

# Wait for publishers to be ready
time.sleep(0.5)

# Arm joint positions (mid position: 0.0 for all)
arm_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll]
gripper_position = [0.0]  # Gripper mid position

# Publish arm trajectory
arm_msg = JointTrajectory()
arm_msg.joint_names = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
point = JointTrajectoryPoint()
point.positions = arm_positions
point.time_from_start.sec = 3
arm_msg.points = [point]
arm_pub.publish(arm_msg)
node.get_logger().info(f'Published arm positions: {arm_positions}')

# Publish gripper position
gripper_msg = Float64MultiArray()
gripper_msg.data = gripper_position
gripper_pub.publish(gripper_msg)
node.get_logger().info(f'Published gripper position: {gripper_position}')

# Exit
rclpy.shutdown()

