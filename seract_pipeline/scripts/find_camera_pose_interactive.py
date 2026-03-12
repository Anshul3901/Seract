#!/usr/bin/env python3
"""
Interactive tool to find camera pose.
This tool helps you determine the camera position relative to base_link.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from rclpy.duration import Duration

class FindCameraPose(Node):
    def __init__(self):
        super().__init__('find_camera_pose')
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('=== Camera Pose Finder ===')
        self.get_logger().info('')
        
    def check_tf_tree(self):
        """Check if camera_link exists in TF tree."""
        self.get_logger().info('1. Checking TF tree...')
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )
            self.get_logger().info('   ✓ Found transform in TF tree!')
            self.get_logger().info(f'   Translation: ({tf.transform.translation.x:.3f}, '
                                 f'{tf.transform.translation.y:.3f}, '
                                 f'{tf.transform.translation.z:.3f})')
            self.get_logger().info(f'   Rotation: ({tf.transform.rotation.x:.3f}, '
                                 f'{tf.transform.rotation.y:.3f}, '
                                 f'{tf.transform.rotation.z:.3f}, '
                                 f'{tf.transform.rotation.w:.3f})')
            return True
        except Exception as e:
            self.get_logger().warn(f'   ✗ camera_link not found in TF tree: {e}')
            return False
    
    def manual_measurement(self):
        """Guide user through manual measurement."""
        self.get_logger().info('')
        self.get_logger().info('2. Manual Measurement Guide:')
        self.get_logger().info('   Measure the camera position relative to base_link:')
        self.get_logger().info('   - X: Distance forward from base (positive = forward)')
        self.get_logger().info('   - Y: Distance left/right from base (positive = left)')
        self.get_logger().info('   - Z: Height above base (positive = up)')
        self.get_logger().info('')
        
        try:
            x = float(input('   Enter X position (meters): '))
            y = float(input('   Enter Y position (meters): '))
            z = float(input('   Enter Z position (meters): '))
            
            self.get_logger().info('')
            self.get_logger().info('   For top-down camera, rotation is typically:')
            self.get_logger().info('   quaternion: (0.0, 0.0, 0.0, 1.0)')
            
            use_default_rot = input('   Use default rotation (identity)? [Y/n]: ').strip().lower()
            if use_default_rot in ['', 'y', 'yes']:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx = float(input('   Enter qx: '))
                qy = float(input('   Enter qy: '))
                qz = float(input('   Enter qz: '))
                qw = float(input('   Enter qw: '))
            
            self.get_logger().info('')
            self.get_logger().info('=== Camera Pose Found ===')
            self.get_logger().info(f'Translation: ({x:.6f}, {y:.6f}, {z:.6f})')
            self.get_logger().info(f'Rotation: ({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})')
            self.get_logger().info('')
            self.get_logger().info('To publish this transform, run:')
            self.get_logger().info(f'ros2 run tf2_ros static_transform_publisher \\')
            self.get_logger().info(f'  {x:.6f} {y:.6f} {z:.6f} \\')
            self.get_logger().info(f'  {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} \\')
            self.get_logger().info(f'  base_link camera_link')
            
            return (x, y, z, qx, qy, qz, qw)
        except (ValueError, EOFError, KeyboardInterrupt):
            self.get_logger().error('   Invalid input or cancelled.')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    finder = FindCameraPose()
    
    try:
        # Check TF tree first
        found = finder.check_tf_tree()
        
        if not found:
            # If not found, guide through manual measurement
            finder.manual_measurement()
        
        # Keep node alive briefly
        rclpy.spin_once(finder, timeout_sec=0.5)
        
    except KeyboardInterrupt:
        pass
    finally:
        finder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

