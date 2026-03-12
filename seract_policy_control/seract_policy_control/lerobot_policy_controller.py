import os, math, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# Delay cv_bridge import to avoid NumPy 2.x compatibility issues
# Will import after environment is set up
import numpy as np
import requests
import cv2

# Optional: only import torch/Lerobot if not in open-loop test
try:
    import torch
    TORCH_OK = True
except Exception:
    TORCH_OK = False

class LeRobotPolicyController(Node):
    def __init__(self):
        super().__init__('lerobot_policy_controller')
        # Declare scalar parameters
        self.declare_parameter('model_checkpoint', '')
        self.declare_parameter('controller_topic', '/so_100_arm_controller/joint_trajectory')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('command_dt', 0.25)
        self.declare_parameter('use_open_loop_test', False)
        self.declare_parameter('use_remote_policy', True)
        self.declare_parameter('policy_url', 'http://127.0.0.1:8009/plan_joints')
        # Declare arm_joint_prefix_whitelist with ignore_override to let YAML set the type
        # This avoids the type conflict when YAML loads it as STRING_ARRAY
        self.declare_parameter('arm_joint_prefix_whitelist', [], ignore_override=True)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_wrist_topic', '/camera2/color/image_raw')  # Wrist camera
        self.declare_parameter('image_encoding', 'rgb8')
        
        # Read scalar parameters
        p = lambda k: self.get_parameter(k).get_parameter_value()
        self.model_checkpoint = p('model_checkpoint').string_value
        self.controller_topic = p('controller_topic').string_value
        self.rate_hz = p('rate_hz').double_value
        self.command_dt = p('command_dt').double_value
        self.use_open_loop_test = p('use_open_loop_test').bool_value
        self.use_remote_policy = self.get_parameter('use_remote_policy').get_parameter_value().bool_value
        self.policy_url = self.get_parameter('policy_url').get_parameter_value().string_value
        arm_whitelist_param = self.get_parameter('arm_joint_prefix_whitelist')
        if arm_whitelist_param.type_ == ParameterType.PARAMETER_STRING_ARRAY:
            self.arm_joint_prefix_whitelist = list(arm_whitelist_param.get_parameter_value().string_array_value)
        else:
            # Convert from byte array if needed
            try:
                byte_val = arm_whitelist_param.get_parameter_value().byte_array_value
                self.arm_joint_prefix_whitelist = [b.decode('utf-8') for b in byte_val] if byte_val else []
            except:
                self.arm_joint_prefix_whitelist = []
        self.image_topic = p('image_topic').string_value
        self.image_wrist_topic = p('image_wrist_topic').string_value
        self.image_encoding = p('image_encoding').string_value
        
        # Read array parameters (handle type conversion from YAML)
        # YAML may set them as BYTE_ARRAY, so we need to convert
        try:
            joint_names_param = self.get_parameter('joint_names')
            if joint_names_param.type_ == ParameterType.PARAMETER_STRING_ARRAY:
                self.joint_names = list(joint_names_param.get_parameter_value().string_array_value)
            else:
                # Convert from byte array or other type
                byte_val = joint_names_param.get_parameter_value().byte_array_value
                self.joint_names = [b.decode('utf-8') for b in byte_val] if byte_val else []
        except:
            self.joint_names = []
        
        try:
            joint_min_param = self.get_parameter('joint_min')
            if joint_min_param.type_ == ParameterType.PARAMETER_DOUBLE_ARRAY:
                self.joint_min = np.array(joint_min_param.get_parameter_value().double_array_value, dtype=np.float64)
            else:
                # Convert from byte array
                byte_val = joint_min_param.get_parameter_value().byte_array_value
                self.joint_min = np.frombuffer(bytes(byte_val), dtype=np.float64) if byte_val else np.array([], dtype=np.float64)
        except:
            self.joint_min = np.array([], dtype=np.float64)
        
        try:
            joint_max_param = self.get_parameter('joint_max')
            if joint_max_param.type_ == ParameterType.PARAMETER_DOUBLE_ARRAY:
                self.joint_max = np.array(joint_max_param.get_parameter_value().double_array_value, dtype=np.float64)
            else:
                # Convert from byte array
                byte_val = joint_max_param.get_parameter_value().byte_array_value
                self.joint_max = np.frombuffer(bytes(byte_val), dtype=np.float64) if byte_val else np.array([], dtype=np.float64)
        except:
            self.joint_max = np.array([], dtype=np.float64)
        
        try:
            test_delta_param = self.get_parameter('test_delta')
            if test_delta_param.type_ == ParameterType.PARAMETER_DOUBLE_ARRAY:
                td = test_delta_param.get_parameter_value().double_array_value
                self.test_delta = np.array(td, dtype=np.float64) if len(td) else None
            else:
                # Convert from byte array
                byte_val = test_delta_param.get_parameter_value().byte_array_value
                self.test_delta = np.frombuffer(bytes(byte_val), dtype=np.float64) if byte_val else None
        except:
            self.test_delta = None

        # Auto-discover joint names if not provided
        if len(self.joint_names) == 0:
            self.get_logger().warn("joint_names parameter empty; will auto-discover from /joint_states")
            self._waiting_for_names = True
        else:
            self._waiting_for_names = False
            n = len(self.joint_names)
            if self.joint_min.size != n or self.joint_max.size != n:
                raise RuntimeError("joint_min/joint_max must match joint_names length")

        # Import cv_bridge here after environment is configured
        # Handle NumPy 2.x compatibility issues gracefully
        self.bridge = None
        import sys
        import io
        try:
            # Suppress stderr temporarily to avoid confusing NumPy 2.x warning traceback
            # The warning is harmless if cv_bridge still imports successfully
            old_stderr = sys.stderr
            sys.stderr = io.StringIO()
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
                # If we get here, import succeeded despite any warnings
                sys.stderr = old_stderr
                self.get_logger().info("cv_bridge imported successfully")
            except (AttributeError, ImportError) as e:
                # Restore stderr to show our error message
                stderr_output = sys.stderr.getvalue()
                sys.stderr = old_stderr
                error_msg = str(e)
                if "_ARRAY_API" in error_msg or "ARRAY_API" in error_msg:
                    self.get_logger().warn(
                        "cv_bridge import failed due to NumPy 2.x incompatibility. "
                        "cv_bridge was compiled with NumPy 1.x. "
                        "The node will start but image processing will not work. "
                        "Solution: Use system Python (conda deactivate) or install numpy<2.0"
                    )
                else:
                    self.get_logger().warn(f"cv_bridge import failed: {error_msg}. Image processing disabled.")
                self.bridge = None
        except Exception as e:
            # Make sure stderr is restored
            if 'old_stderr' in locals():
                sys.stderr = old_stderr
            self.get_logger().warn(f"Failed to import cv_bridge: {e}. Image processing disabled.")
            self.bridge = None
        self.last_image = None
        self.last_wrist_image = None
        self.last_joints = None
        self._scene_image_received = False
        self._wrist_image_received = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.image_wrist_sub = self.create_subscription(Image, self.image_wrist_topic, self.on_wrist_image, qos)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        
        self.get_logger().info(f"Subscribed to scene camera: {self.image_topic}")
        self.get_logger().info(f"Subscribed to wrist camera: {self.image_wrist_topic}")

        self.pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)

        self.timer = self.create_timer(1.0/max(self.rate_hz,1.0), self.tick)

        # Lazy-load policy (only if not using remote policy)
        self.policy = None
        self.device = 'cuda' if TORCH_OK and torch.cuda.is_available() else 'cpu'
        if not self.use_open_loop_test and not self.use_remote_policy:
            self._load_policy()

        self.get_logger().info(f"LeRobotPolicyController started. controller_topic={self.controller_topic}, rate={self.rate_hz}Hz")

    def _load_policy(self):
        try:
            from lerobot.models import ACTPolicy  # adapt to your API
            self.policy = ACTPolicy.load_from_checkpoint(self.model_checkpoint, map_location=self.device)
            self.policy.eval()
            if TORCH_OK:
                self.policy.to(self.device)
            self.get_logger().info(f"Loaded policy from {self.model_checkpoint} on {self.device}")
        except Exception as e:
            self.get_logger().error(f"Failed to load policy: {e}")
            self.use_open_loop_test = True

    def on_image(self, msg: Image):
        if self.bridge is None:
            # Only warn once to avoid spam
            if not hasattr(self, '_cv_bridge_warned'):
                self.get_logger().warn("cv_bridge not available, cannot process images")
                self._cv_bridge_warned = True
            return
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)
            if not self._scene_image_received:
                self.get_logger().info(f"Scene camera image received: {self.last_image.shape}")
                self._scene_image_received = True
        except Exception as e:
            self.get_logger().warn(f"Scene image convert failed: {e}")
    
    def on_wrist_image(self, msg: Image):
        if self.bridge is None:
            return
        try:
            self.last_wrist_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)
            if not self._wrist_image_received:
                self.get_logger().info(f"Wrist camera image received: {self.last_wrist_image.shape}")
                self._wrist_image_received = True
        except Exception as e:
            self.get_logger().warn(f"Wrist image convert failed: {e}")

    def on_joints(self, msg: JointState):
        if getattr(self, "_waiting_for_names", False):
            # adopt names from first joint_states message
            self.joint_names = list(msg.name)
            
            # Filter joints by whitelist if provided
            if self.arm_joint_prefix_whitelist:
                keep = []
                for n in self.joint_names:
                    if any(n.startswith(pref) for pref in self.arm_joint_prefix_whitelist):
                        keep.append(n)
                if keep:
                    self.joint_names = keep
                    self.get_logger().info(f"Filtered joint_names to arm only: {self.joint_names}")
            
            n = len(self.joint_names)
            if self.joint_min.size == 0:
                self.joint_min = np.full(n, -np.pi, dtype=np.float64)
            if self.joint_max.size == 0:
                self.joint_max = np.full(n, np.pi, dtype=np.float64)
            self.get_logger().info(f"Auto-discovered joint_names: {self.joint_names}")
            self._waiting_for_names = False
        
        # map joint_states into our ordered joint_names
        name_to_pos = {n:p for n,p in zip(msg.name, msg.position)}
        try:
            ordered = []
            for n in self.joint_names:
                if n in name_to_pos:
                    ordered.append(name_to_pos[n])
                else:
                    raise KeyError(n)
            self.last_joints = np.array(ordered, dtype=np.float64)
            
            # Log first successful joint state reception
            if not hasattr(self, '_joints_received_logged'):
                self.get_logger().info(f"Joint states received: {len(self.last_joints)} joints, values: {self.last_joints}")
                self._joints_received_logged = True
        except KeyError as e:
            if not hasattr(self, '_joint_missing_warned'):
                self.get_logger().warn(f"Joint '{e}' not found in joint_states. Available: {list(name_to_pos.keys())}")
                self._joint_missing_warned = True
            return

    def _clamp(self, q):
        return np.minimum(self.joint_max, np.maximum(self.joint_min, q))

    def tick(self):
        if self.last_joints is None:
            return

        # Decide command
        if self.use_open_loop_test:
            delta = self.test_delta if self.test_delta is not None else np.zeros_like(self.last_joints)
            q_cmd = self._clamp(self.last_joints + delta)
        elif self.use_remote_policy:
            if self.bridge is None:
                # Only error once to avoid spam
                if not hasattr(self, '_remote_policy_error_shown'):
                    self.get_logger().error("Cannot use remote policy: cv_bridge not available (NumPy 2.x incompatibility)")
                    self._remote_policy_error_shown = True
                return
            
            # Check if we have both images
            if self.last_image is None:
                if not hasattr(self, '_missing_scene_warned'):
                    self.get_logger().warn("Scene camera image not available, waiting...")
                    self._missing_scene_warned = True
                return
            
            if self.last_wrist_image is None:
                if not hasattr(self, '_missing_wrist_warned'):
                    self.get_logger().warn("Wrist camera image not available, waiting...")
                    self._missing_wrist_warned = True
                return
            
            try:
                # Both images are available - prepare them for policy server
                img_scene = self.last_image
                img_wrist = self.last_wrist_image
                
                # Validate image shapes
                if len(img_scene.shape) != 3 or img_scene.shape[2] != 3:
                    raise RuntimeError(f"Scene image has invalid shape: {img_scene.shape}, expected (H, W, 3)")
                if len(img_wrist.shape) != 3 or img_wrist.shape[2] != 3:
                    raise RuntimeError(f"Wrist image has invalid shape: {img_wrist.shape}, expected (H, W, 3)")
                
                # Downsample if needed to keep payload small (target: max 640x480)
                target_pixels = 640 * 480
                scene_pixels = img_scene.shape[0] * img_scene.shape[1]
                wrist_pixels = img_wrist.shape[0] * img_wrist.shape[1]
                
                if scene_pixels > target_pixels:
                    # Downsample by factor of 2
                    small_scene = img_scene[::2, ::2, :]
                else:
                    small_scene = img_scene
                
                if wrist_pixels > target_pixels:
                    # Downsample by factor of 2
                    small_wrist = img_wrist[::2, ::2, :]
                else:
                    small_wrist = img_wrist
                
                # Convert RGB to BGR for OpenCV encoding (OpenCV uses BGR)
                # Note: cv_bridge gives us RGB, but cv2.imencode expects BGR
                scene_bgr = cv2.cvtColor(small_scene, cv2.COLOR_RGB2BGR)
                wrist_bgr = cv2.cvtColor(small_wrist, cv2.COLOR_RGB2BGR)
                
                # Encode as JPEG with quality setting
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]  # Good quality, reasonable size
                ok_scene, buf_scene = cv2.imencode(".jpg", scene_bgr, encode_params)
                ok_wrist, buf_wrist = cv2.imencode(".jpg", wrist_bgr, encode_params)
                
                if not ok_scene:
                    raise RuntimeError("cv2.imencode failed for scene image")
                if not ok_wrist:
                    raise RuntimeError("cv2.imencode failed for wrist image")
                
                # Prepare payload for policy server
                # Format: {"joints": [...], "image_scene_jpeg": "hex_string", "image_wrist_jpeg": "hex_string"}
                payload = {
                    "joints": self.last_joints.tolist(),
                    "image_scene_jpeg": buf_scene.tobytes().hex(),  # Scene camera (RealSense)
                    "image_wrist_jpeg": buf_wrist.tobytes().hex(),  # Wrist camera (USB webcam)
                }
                
                # Send to policy server
                r = requests.post(self.policy_url, json=payload, timeout=0.2)
                r.raise_for_status()
                js = r.json()
                
                # Parse response - policy server returns {"mode": "delta"/"abs", "joints": [...]}
                if "joints" not in js:
                    raise RuntimeError("policy response missing 'joints' field")
                
                response_mode = js.get("mode", "delta")  # Default to delta if not specified
                joints_response = np.array(js["joints"], dtype=np.float64)
                
                # Validate joint count
                if len(joints_response) != len(self.last_joints):
                    raise RuntimeError(f"Policy response has {len(joints_response)} joints, expected {len(self.last_joints)}")
                
                # Apply based on mode
                if response_mode == "abs":
                    # Absolute positions
                    q_cmd = joints_response
                else:
                    # Delta mode: add to current positions
                    q_cmd = self.last_joints + joints_response
                
                q_cmd = self._clamp(q_cmd)
                
                # Log command details occasionally
                if not hasattr(self, '_last_cmd_log') or time.time() - self._last_cmd_log > 5.0:
                    self.get_logger().debug(
                        f"Policy command: mode={response_mode}, "
                        f"current={self.last_joints}, "
                        f"response={joints_response}, "
                        f"command={q_cmd}"
                    )
                    self._last_cmd_log = time.time()
                
                # Log success occasionally (not every tick to avoid spam)
                if not hasattr(self, '_last_success_log') or time.time() - self._last_success_log > 5.0:
                    self.get_logger().debug(f"Policy server call successful. Scene: {small_scene.shape}, Wrist: {small_wrist.shape}")
                    self._last_success_log = time.time()
                    
            except requests.exceptions.RequestException as e:
                if not hasattr(self, '_last_request_error') or time.time() - self._last_request_error > 2.0:
                    self.get_logger().warn(f"Policy server request failed: {e}; holding position")
                    self._last_request_error = time.time()
                q_cmd = self._clamp(self.last_joints)
            except Exception as e:
                if not hasattr(self, '_last_error') or time.time() - self._last_error > 2.0:
                    self.get_logger().warn(f"Remote policy processing failed: {e}; holding position")
                    self._last_error = time.time()
                q_cmd = self._clamp(self.last_joints)
        else:
            # local-inprocess policy path (only used if you later install lerobot in this interpreter)
            q_cmd = self._clamp(self.last_joints)

        # Publish a single-point trajectory
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_cmd.tolist()]
        pt.time_from_start.sec = int(self.command_dt)
        pt.time_from_start.nanosec = int((self.command_dt - int(self.command_dt))*1e9)
        traj.points.append(pt)

        self.pub.publish(traj)

def main():
    rclpy.init()
    node = LeRobotPolicyController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

