#!/usr/bin/env python3

import time
import os
import requests
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from seract_msgs.action import ExecutePrimitive
from seract_pipeline.logging_utils import log_run
import tf2_ros
from tf2_ros import TransformException
import tf_transformations
import numpy as np


class Pipeline(Node):
    def __init__(self):
        super().__init__('pipeline')
        
        # Declare parameters from pipeline.yaml
        self.declare_parameter('policy_url', 'http://127.0.0.1:8008/plan_grasp')
        self.declare_parameter('quality_threshold', 0.6)
        self.declare_parameter('snapshot_topic', '/camera/color/image_raw')
        self.declare_parameter('snapshot_path', '/tmp/frame.jpg')
        self.declare_parameter('use_mock_primitives', True)
        
        # Declare pose parameters (ROS2 doesn't support dicts directly, so we use defaults)
        # These will be loaded from YAML if provided, otherwise use defaults
        self.declare_parameter('accept_pose.frame', 'base_link')
        self.declare_parameter('accept_pose.xyz', [0.40, 0.25, 0.10])
        self.declare_parameter('accept_pose.quat', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('reject_pose.frame', 'base_link')
        self.declare_parameter('reject_pose.xyz', [0.40, -0.25, 0.10])
        self.declare_parameter('reject_pose.quat', [0.0, 0.0, 0.0, 1.0])
        
        # Get parameters
        self.policy_url = self.get_parameter('policy_url').get_parameter_value().string_value
        self.quality_threshold = self.get_parameter('quality_threshold').get_parameter_value().double_value
        self.snapshot_topic = self.get_parameter('snapshot_topic').get_parameter_value().string_value
        self.snapshot_path = self.get_parameter('snapshot_path').get_parameter_value().string_value
        self.use_mock_primitives = self.get_parameter('use_mock_primitives').get_parameter_value().bool_value
        
        # Build pose configs from parameters
        self.accept_pose_cfg = {
            'frame': self.get_parameter('accept_pose.frame').get_parameter_value().string_value,
            'xyz': self.get_parameter('accept_pose.xyz').get_parameter_value().double_array_value,
            'quat': self.get_parameter('accept_pose.quat').get_parameter_value().double_array_value
        }
        self.reject_pose_cfg = {
            'frame': self.get_parameter('reject_pose.frame').get_parameter_value().string_value,
            'xyz': self.get_parameter('reject_pose.xyz').get_parameter_value().double_array_value,
            'quat': self.get_parameter('reject_pose.quat').get_parameter_value().double_array_value
        }
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # Create TF buffer and listener for pose transformations
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_link = 'base_link'
        
        # Subscribe to snapshot topic
        self.image_subscription = self.create_subscription(
            Image,
            self.snapshot_topic,
            self.image_callback,
            10
        )
        
        # Create action client for primitive execution
        self.action_client = ActionClient(self, ExecutePrimitive, '/execute_primitive')
        
        # Robust wait for action server
        for t in (5.0, 10.0, 15.0):
            if self.action_client.wait_for_server(timeout_sec=t):
                break
            self.get_logger().warn(f"Action server /execute_primitive not up yet (waited {t}s)...")
        else:
            self.get_logger().error("Action server /execute_primitive not available; skipping this cycle.")
        
        self.get_logger().info(f'Pipeline initialized')
        self.get_logger().info(f'  policy_url: {self.policy_url}')
        self.get_logger().info(f'  quality_threshold: {self.quality_threshold}')
        self.get_logger().info(f'  snapshot_topic: {self.snapshot_topic}')
        self.get_logger().info(f'  snapshot_path: {self.snapshot_path}')
        self.get_logger().info(f'  use_mock_primitives: {self.use_mock_primitives}')
    
    def image_callback(self, msg):
        """Save latest image to snapshot_path."""
        try:
            # Convert ROS Image to OpenCV format
            # Camera publishes rgb8, convert to bgr8 for OpenCV
            cv_image_rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            # Convert RGB to BGR for OpenCV
            cv_image = cv2.cvtColor(cv_image_rgb, cv2.COLOR_RGB2BGR)
            
            # Save to snapshot_path
            cv2.imwrite(self.snapshot_path, cv_image)
            self.latest_image = cv_image
            self.image_received = True
            
            self.get_logger().debug(f'Saved snapshot to {self.snapshot_path}')
        except Exception as e:
            self.get_logger().error(f'Error saving snapshot: {e}')
    
    def call_policy(self, rgb_path):
        """
        POST to policy_url with body {"rgb_path": rgb_path}
        
        Args:
            rgb_path: Path to the image file to send to the policy server
        
        Returns:
            dict: Response from policy server or None on error
        """
        try:
            response = requests.post(
                self.policy_url,
                json={"rgb_path": rgb_path},
                timeout=10.0
            )
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Policy call failed: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error calling policy: {e}')
            return None
    
    def transform_to_base_link(self, ps):
        """
        Transform PoseStamped to base_link frame.
        
        If the source frame doesn't exist in TF tree, assume it's camera_link
        and apply a default transform (camera mounted above looking down).
        
        Args:
            ps: PoseStamped in any frame
        
        Returns:
            PoseStamped: Transformed pose in base_link frame
        """
        if ps.header.frame_id == self.base_link:
            return ps
        
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link,                 # target frame
                ps.header.frame_id,             # source frame
                rclpy.time.Time(),              # latest
                timeout=Duration(seconds=1.0)
            )
            
            # Manual transform using tf_transformations
            result = PoseStamped()
            result.header.frame_id = self.base_link
            result.header.stamp = tf.header.stamp
            
            # Get transform translation and rotation
            tf_trans = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
            tf_rot = [tf.transform.rotation.x, tf.transform.rotation.y, 
                     tf.transform.rotation.z, tf.transform.rotation.w]
            
            # Get source pose position and rotation
            src_pos = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            src_rot = [ps.pose.orientation.x, ps.pose.orientation.y,
                      ps.pose.orientation.z, ps.pose.orientation.w]
            
            # Transform position: rotate then translate
            src_pos_array = np.array(src_pos)
            tf_rot_matrix = tf_transformations.quaternion_matrix(tf_rot)[:3, :3]
            rotated_pos = tf_rot_matrix @ src_pos_array
            result_pos = rotated_pos + np.array(tf_trans)
            
            # Transform orientation: multiply quaternions
            result_quat = tf_transformations.quaternion_multiply(tf_rot, src_rot)
            
            # Set result pose
            result.pose.position.x = float(result_pos[0])
            result.pose.position.y = float(result_pos[1])
            result.pose.position.z = float(result_pos[2])
            result.pose.orientation.x = float(result_quat[0])
            result.pose.orientation.y = float(result_quat[1])
            result.pose.orientation.z = float(result_quat[2])
            result.pose.orientation.w = float(result_quat[3])
            
            return result
        except TransformException as ex:
            self.get_logger().warn(f"TF transform failed {ps.header.frame_id}->{self.base_link}: {ex}")
            # Fallback: if camera_link doesn't exist, use measured camera pose
            # From TF tree: camera is at (0.4, 0.0, 0.6) in base_link with identity rotation
            if ps.header.frame_id == 'camera_link':
                self.get_logger().warn("Using fallback transform: camera_link at (0.4, 0.0, 0.6) in base_link")
                result = PoseStamped()
                result.header.frame_id = self.base_link
                result.header.stamp = self.get_clock().now().to_msg()
                # Camera is at (0.4, 0.0, 0.6) in base_link, looking down
                # Camera z points down, so camera z = -distance from camera
                # In base_link: z_base = camera_z_base + camera_z_cam (camera z is negative)
                camera_x_base = 0.4  # Camera x position in base_link
                camera_y_base = 0.0  # Camera y position in base_link
                camera_z_base = 0.6  # Camera z position in base_link (height above base)
                result.pose.position.x = ps.pose.position.x + camera_x_base
                result.pose.position.y = ps.pose.position.y + camera_y_base
                result.pose.position.z = camera_z_base + ps.pose.position.z  # Flip z (camera z is negative)
                # Keep orientation (will be normalized and set to tool_down later)
                result.pose.orientation = ps.pose.orientation
                return result
            # For other frames, just change frame_id and hope it works
            ps.header.frame_id = self.base_link
            return ps
        except Exception as ex:
            self.get_logger().error(f"Transform calculation failed: {ex}")
            # Fallback: change frame_id
            ps.header.frame_id = self.base_link
            return ps
    
    def clamp_pose(self, ps):
        """
        Clamp pose to reachable workspace bounds.
        
        Args:
            ps: PoseStamped in base_link
        
        Returns:
            PoseStamped: Clamped pose
        """
        p = ps.pose.position
        # Clamp to reachable workspace (more conservative bounds to avoid edges)
        # Keep poses away from edges for better planning success
        p.x = max(0.24, min(0.32, p.x))  # Slightly tighter than primitive_server
        p.y = max(-0.15, min(0.15, p.y))  # Slightly tighter than primitive_server
        p.z = max(0.33, min(0.43, p.z))  # Slightly tighter than primitive_server
        return ps
    
    def pose_from_cfg(self, cfg):
        """
        Build PoseStamped from config dict {frame, xyz, quat}.
        
        Args:
            cfg: dict with keys 'frame', 'xyz' (list of 3 floats), 'quat' (list of 4 floats)
        
        Returns:
            PoseStamped: ROS message
        """
        pose = PoseStamped()
        pose.header.frame_id = cfg.get('frame', 'base_link')
        pose.header.stamp = self.get_clock().now().to_msg()
        
        xyz = cfg.get('xyz', [0.0, 0.0, 0.0])
        quat = cfg.get('quat', [0.0, 0.0, 0.0, 1.0])
        
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        
        return pose
    
    def send_primitive(self, name, pose, approach=0.08, retreat=0.12, speed=0.5):
        """
        Send primitive action to /execute_primitive.
        
        Args:
            name: Primitive name (e.g., "pick", "place")
            pose: PoseStamped target pose
            approach: Approach offset in meters (default 0.08)
            retreat: Retreat offset in meters (default 0.12)
            speed: Speed scale (default 0.5)
        
        Returns:
            bool: True if successful, False otherwise
        """
        # Create goal
        goal_msg = ExecutePrimitive.Goal()
        goal_msg.primitive_name = name
        goal_msg.target_pose = pose
        goal_msg.approach_offset = [0.0, 0.0, -approach]  # Approach from above
        goal_msg.retreat_offset = [0.0, 0.0, retreat]  # Retreat upward
        goal_msg.speed_scale = speed
        
        # Send goal
        self.get_logger().info(f'Sending {name} primitive to pose: {pose.pose.position}')
        try:
            send_future = self.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
            goal_handle = send_future.result()
            if goal_handle is None:
                self.get_logger().error("Failed to get goal handle (future returned None)")
                return False
            if not goal_handle.accepted:
                self.get_logger().warn("Goal rejected by server")
                return False
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
            if result_future.result() is None:
                self.get_logger().error("Failed to get result (future returned None)")
                return False
            result = result_future.result().result
            self.get_logger().info(f"Primitive result: {getattr(result, 'status', 'unknown')}")
            
            if result.success:
                self.get_logger().info(f'{name} primitive completed successfully')
            else:
                self.get_logger().warn(f'{name} primitive failed: {result.error_code}')
            
            return result.success
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            return False
    
    def run_once(self):
        """
        Run one pipeline cycle:
        1. Wait up to 3s for a frame, save to snapshot_path
        2. Call policy with body {"rgb_path": snapshot_path, "mock": use_mock_primitives}
        3. If success: send pick on returned grasp pose
        4. Route to ACCEPT if quality >= threshold, else REJECT; send place
        5. Call logging_utils.log_run() with metadata
        """
        self.get_logger().info('Starting pipeline cycle')
        
        # Wait up to 3s for a frame
        self.image_received = False
        timeout = 3.0
        start_time = time.time()
        
        while not self.image_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if not self.image_received:
            self.get_logger().warn('No frame received within timeout')
            return False
        
        # Ensure snapshot is saved
        if not os.path.exists(self.snapshot_path):
            self.get_logger().error(f'Snapshot not saved to {self.snapshot_path}')
            return False
        
        # Call policy (only if not using mock primitives)
        if self.use_mock_primitives:
            self.get_logger().info('Skipping policy call (use_mock_primitives=true)')
            # Create mock response for testing
            policy_resp = {
                'quality': 0.7,
                'position': {'x': 0.30, 'y': 0.00, 'z': 0.36},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'frame_id': 'base_link'
            }
        else:
            # Call real policy server
            self.get_logger().info(f'Calling policy with rgb_path={self.snapshot_path}')
            policy_resp = self.call_policy(self.snapshot_path)
            
            if policy_resp is None:
                self.get_logger().error('Policy call failed')
                return False
        
        # Check for error in response
        if 'error' in policy_resp:
            self.get_logger().error(f'Policy returned error: {policy_resp["error"]}')
            return False
        
        # Extract grasp pose and quality
        quality = policy_resp.get('quality', 0.0)
        position = policy_resp.get('position', {})
        orientation = policy_resp.get('orientation', {})
        frame_id = policy_resp.get('frame_id', 'camera_link')
        
        # Build grasp pose in camera_link frame
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = frame_id
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position.x = float(position.get('x', 0.0))
        grasp_pose.pose.position.y = float(position.get('y', 0.0))
        grasp_pose.pose.position.z = float(position.get('z', 0.0))
        grasp_pose.pose.orientation.x = float(orientation.get('x', 0.0))
        grasp_pose.pose.orientation.y = float(orientation.get('y', 0.0))
        grasp_pose.pose.orientation.z = float(orientation.get('z', 0.0))
        grasp_pose.pose.orientation.w = float(orientation.get('w', 1.0))
        
        self.get_logger().info(f'Policy returned quality: {quality}, pose in {frame_id}: {grasp_pose.pose.position}')
        
        # Transform to base_link and clamp to reachable workspace
        grasp_pose = self.transform_to_base_link(grasp_pose)
        grasp_pose = self.clamp_pose(grasp_pose)
        
        self.get_logger().info(f'Transformed and clamped pose in base_link: {grasp_pose.pose.position}')
        
        # Send pick primitive
        pick_success = self.send_primitive("pick", grasp_pose)
        
        # Route to ACCEPT or REJECT based on quality threshold
        if quality >= self.quality_threshold:
            decision = "ACCEPT"
            place_pose = self.pose_from_cfg(self.accept_pose_cfg)
        else:
            decision = "REJECT"
            place_pose = self.pose_from_cfg(self.reject_pose_cfg)
        
        self.get_logger().info(f'Decision: {decision} (quality={quality}, threshold={self.quality_threshold})')
        
        # Send place primitive
        place_success = self.send_primitive("place", place_pose)
        
        # Log run
        log_data = {
            "policy_resp": policy_resp,
            "pick_success": pick_success,
            "place_success": place_success,
            "quality_threshold": self.quality_threshold,
            "decision": decision,
            "snapshot_path": self.snapshot_path,
            "timestamp": time.time()
        }
        
        # Determine output directory (use environment variable or default)
        out_dir = os.getenv('PIPELINE_LOG_DIR', '/tmp/pipeline_logs')
        log_dir = log_run(out_dir, log_data)
        self.get_logger().info(f'Logged run to: {log_dir}')
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    pipeline = Pipeline()
    
    try:
        # Run once
        pipeline.run_once()
        
        # Keep spinning for a bit to handle any remaining callbacks
        rclpy.spin_once(pipeline, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

