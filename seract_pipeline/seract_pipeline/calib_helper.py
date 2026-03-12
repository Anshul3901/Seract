#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs


class CalibHelper(Node):
    def __init__(self):
        super().__init__('calib_helper')
        
        # Declare parameters
        self.declare_parameter('object_height_m', 0.02)
        self.declare_parameter('table_z_in_base_m', 0.0)
        self.declare_parameter('assume_topdown', True)
        
        # Get parameters
        self.object_height_m = self.get_parameter('object_height_m').get_parameter_value().double_value
        self.table_z_in_base_m = self.get_parameter('table_z_in_base_m').get_parameter_value().double_value
        self.assume_topdown = self.get_parameter('assume_topdown').get_parameter_value().bool_value
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Calibration helper initialized')
        self.get_logger().info(f'  object_height_m: {self.object_height_m}')
        self.get_logger().info(f'  table_z_in_base_m: {self.table_z_in_base_m}')
        self.get_logger().info(f'  assume_topdown: {self.assume_topdown}')
    
    def wait_for_grasp_pose(self):
        """
        Wait for a policy grasp pose in camera_link frame.
        Prompts user to input values from policy server response.
        """
        self.get_logger().info('=== Getting Grasp Pose ===')
        self.get_logger().info('Please run the policy server and get a grasp pose response.')
        self.get_logger().info('Enter the position and orientation from the policy response:')
        self.get_logger().info('')
        
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'camera_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Prompt user for grasp pose values
        try:
            self.get_logger().info('Enter grasp position (in camera_link frame):')
            x = float(input('  X (meters): '))
            y = float(input('  Y (meters): '))
            z = float(input('  Z (meters): '))
            
            if self.assume_topdown:
                # Top-down: assume z-axis normal (pointing down)
                self.get_logger().info('Using top-down orientation (z-axis down)')
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                self.get_logger().info('Enter grasp orientation (quaternion in camera_link frame):')
                qx = float(input('  qx: '))
                qy = float(input('  qy: '))
                qz = float(input('  qz: '))
                qw = float(input('  qw: '))
            
            grasp_pose.pose.position.x = x
            grasp_pose.pose.position.y = y
            grasp_pose.pose.position.z = z
            grasp_pose.pose.orientation.x = qx
            grasp_pose.pose.orientation.y = qy
            grasp_pose.pose.orientation.z = qz
            grasp_pose.pose.orientation.w = qw
            
        except (ValueError, EOFError):
            self.get_logger().error('Invalid input. Using default values.')
            self.get_logger().error('Please run again and provide valid values.')
            # Use defaults
            grasp_pose.pose.position.x = 0.0
            grasp_pose.pose.position.y = 0.0
            grasp_pose.pose.position.z = 0.5
            grasp_pose.pose.orientation.x = 0.0
            grasp_pose.pose.orientation.y = 0.0
            grasp_pose.pose.orientation.z = 0.0
            grasp_pose.pose.orientation.w = 1.0
        
        return grasp_pose
    
    def compute_transform(self, grasp_pose_camera, table_xy_base):
        """
        Solve PnP-lite: compute translation-only transform.
        
        The grasp pose is in camera_link frame.
        We want to find the transform from base_link to camera_link such that:
        - The grasp z position in base_link = table_z_in_base_m - object_height_m
        
        Args:
            grasp_pose_camera: PoseStamped in camera_link frame
            table_xy_base: [x, y] position on table in base_link frame
        
        Returns:
            tuple: (tx, ty, tz) translation from base_link to camera_link
        """
        # Get grasp position in camera frame
        grasp_x_cam = grasp_pose_camera.pose.position.x
        grasp_y_cam = grasp_pose_camera.pose.position.y
        grasp_z_cam = grasp_pose_camera.pose.position.z
        
        # Target position in base frame
        target_x_base = table_xy_base[0]
        target_y_base = table_xy_base[1]
        target_z_base = self.table_z_in_base_m - self.object_height_m
        
        # If assume_topdown, we can simplify:
        # The camera is looking down, so the grasp z in camera is the distance from camera
        # We want: base_z = table_z - object_height
        # So: camera_z (in base) = target_z_base
        # But we need to account for the camera's position
        
        # For a simple translation-only transform:
        # If we assume the camera is mounted above looking down:
        # - The grasp point in camera frame is at (grasp_x_cam, grasp_y_cam, grasp_z_cam)
        # - We want this point to be at (target_x_base, target_y_base, target_z_base) in base frame
        
        # Simple approach: assume camera_link origin should be at:
        # base_link position = (target_x_base - grasp_x_cam, target_y_base - grasp_y_cam, target_z_base - grasp_z_cam)
        # But this assumes no rotation, which is only valid if assume_topdown
        
        if self.assume_topdown:
            # For top-down, the camera z-axis points down
            # The grasp z in camera is the distance from camera to object
            # We want the object to be at table_z - object_height in base frame
            
            # Simple translation: camera_link origin in base_link
            # If grasp is at (grasp_x_cam, grasp_y_cam, grasp_z_cam) in camera_link
            # and we want it at (target_x_base, target_y_base, target_z_base) in base_link
            # Then: base_link_to_camera_link = (target - grasp) in base frame
            
            # But we need to account for rotation. For top-down:
            # Camera z points down, so we need to transform
            
            # Simplified: assume camera_link and base_link have same orientation (or small rotation)
            # Then translation is: t = target_base - grasp_camera (in base frame)
            
            # For top-down camera, if z points down:
            # grasp_z_cam is positive downward distance
            # In base frame, we want the object at target_z_base
            # So camera_link origin z in base = target_z_base + grasp_z_cam
            
            tx = target_x_base - grasp_x_cam
            ty = target_y_base - grasp_y_cam
            tz = target_z_base + grasp_z_cam  # Add because camera z points down
            
        else:
            # More complex: would need to account for rotation
            # For now, use same simple approach
            tx = target_x_base - grasp_x_cam
            ty = target_y_base - grasp_y_cam
            tz = target_z_base + grasp_z_cam
        
        return (tx, ty, tz)
    
    def run(self):
        """Run the calibration helper."""
        self.get_logger().info('=== Calibration Helper ===')
        self.get_logger().info('This tool helps calibrate the camera_link to base_link transform.')
        
        # Wait for grasp pose
        grasp_pose_camera = self.wait_for_grasp_pose()
        self.get_logger().info(f'Grasp pose in camera_link:')
        self.get_logger().info(f'  position: ({grasp_pose_camera.pose.position.x:.3f}, '
                              f'{grasp_pose_camera.pose.position.y:.3f}, '
                              f'{grasp_pose_camera.pose.position.z:.3f})')
        
        # Prompt user for table XY position
        self.get_logger().info('')
        self.get_logger().info('Please place a flat puck at a known XY position on the table.')
        self.get_logger().info(f'Table height in base_link: {self.table_z_in_base_m:.3f} m')
        self.get_logger().info(f'Object height: {self.object_height_m:.3f} m')
        
        # Get table XY from user (in practice, could be from input or measurement)
        try:
            table_x = float(input('Enter table X position in base_link (meters): '))
            table_y = float(input('Enter table Y position in base_link (meters): '))
        except (ValueError, EOFError):
            self.get_logger().error('Invalid input. Using default position (0.4, 0.0)')
            table_x = 0.4
            table_y = 0.0
        
        table_xy_base = [table_x, table_y]
        
        # Compute transform
        self.get_logger().info('Computing transform...')
        tx, ty, tz = self.compute_transform(grasp_pose_camera, table_xy_base)
        
        # Print suggested static_transform_publisher command
        self.get_logger().info('')
        self.get_logger().info('=== Suggested Transform ===')
        self.get_logger().info('Run this command to publish the transform:')
        self.get_logger().info('')
        
        # For top-down, rotation is typically identity or small
        if self.assume_topdown:
            # Top-down: typically identity quaternion or small rotation
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            # Could compute rotation if needed
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        
        cmd = (f'ros2 run tf2_ros static_transform_publisher '
               f'{tx:.6f} {ty:.6f} {tz:.6f} '
               f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} '
               f'base_link camera_link')
        
        self.get_logger().info(cmd)
        self.get_logger().info('')
        self.get_logger().info('Or add to a launch file:')
        self.get_logger().info('')
        self.get_logger().info(f'  <node pkg="tf2_ros" exec="static_transform_publisher" name="base_to_camera">')
        self.get_logger().info(f'    <param name="x" value="{tx:.6f}"/>')
        self.get_logger().info(f'    <param name="y" value="{ty:.6f}"/>')
        self.get_logger().info(f'    <param name="z" value="{tz:.6f}"/>')
        self.get_logger().info(f'    <param name="qx" value="{qx:.6f}"/>')
        self.get_logger().info(f'    <param name="qy" value="{qy:.6f}"/>')
        self.get_logger().info(f'    <param name="qz" value="{qz:.6f}"/>')
        self.get_logger().info(f'    <param name="qw" value="{qw:.6f}"/>')
        self.get_logger().info(f'    <param name="frame_id" value="base_link"/>')
        self.get_logger().info(f'    <param name="child_frame_id" value="camera_link"/>')
        self.get_logger().info(f'  </node>')
        self.get_logger().info('')
        
        self.get_logger().info('Transform computed:')
        self.get_logger().info(f'  Translation: ({tx:.6f}, {ty:.6f}, {tz:.6f})')
        self.get_logger().info(f'  Rotation (quaternion): ({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})')


def main(args=None):
    rclpy.init(args=args)
    
    calib_helper = CalibHelper()
    
    try:
        calib_helper.run()
        
        # Keep node alive briefly
        rclpy.spin_once(calib_helper, timeout_sec=0.5)
        
    except KeyboardInterrupt:
        pass
    finally:
        calib_helper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

