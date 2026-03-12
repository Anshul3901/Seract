#!/usr/bin/env python3
"""
Rerun visualizer for ROS2 topics
Visualizes joint states, camera images, and robot transforms
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from tf2_ros import TransformListener, Buffer
import rerun as rr
import numpy as np
import cv2
from cv_bridge import CvBridge


class RerunVisualizer(Node):
    def __init__(self):
        super().__init__('rerun_visualizer')
        
        # Initialize rerun
        rr.init("ROS2 Robot Visualization", spawn=True)
        
        self.bridge = CvBridge()
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.scene_image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.scene_image_callback, 10
        )
        
        self.wrist_image_sub = self.create_subscription(
            Image, '/camera2/color/image_raw', self.wrist_image_callback, 10
        )
        
        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Rerun visualizer started")
        self.get_logger().info("Visualizing:")
        self.get_logger().info("  - Joint states: /joint_states")
        self.get_logger().info("  - Scene camera: /camera/camera/color/image_raw")
        self.get_logger().info("  - Wrist camera: /camera2/color/image_raw")
    
    def joint_state_callback(self, msg: JointState):
        """Log joint states to rerun"""
        try:
            # Create a dictionary of joint positions
            joint_data = {}
            for name, position in zip(msg.name, msg.position):
                joint_data[name] = position
            
            # Log joint positions as scalars
            for name, position in joint_data.items():
                rr.log(f"joints/{name}", rr.Scalar(position))
            
            # Also log as a bar chart
            joint_names = list(joint_data.keys())
            joint_positions = [joint_data[name] for name in joint_names]
            rr.log("joints/bar_chart", rr.BarChart(joint_positions, labels=joint_names))
            
        except Exception as e:
            self.get_logger().warn(f"Error logging joint states: {e}")
    
    def scene_image_callback(self, msg: Image):
        """Log scene camera image to rerun"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # Rerun expects images in [H, W, 3] format
            rr.log("camera/scene", rr.Image(cv_image))
        except Exception as e:
            self.get_logger().warn(f"Error logging scene image: {e}")
    
    def wrist_image_callback(self, msg: Image):
        """Log wrist camera image to rerun"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            rr.log("camera/wrist", rr.Image(cv_image))
        except Exception as e:
            self.get_logger().warn(f"Error logging wrist image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RerunVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


