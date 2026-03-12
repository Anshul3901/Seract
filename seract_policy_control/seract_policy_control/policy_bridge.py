import rclpy
import requests
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .safety import Safety

# Optional cv_bridge import (may fail with NumPy 2.x)
try:
    from cv_bridge import CvBridge
    import cv2
    CV_BRIDGE_AVAILABLE = True
except ImportError as e:
    CV_BRIDGE_AVAILABLE = False
    cv_bridge_error = str(e)

# CRITICAL: Joint order must match controller's expected order (from controllers.yaml)
# Controller expects: ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
DEFAULT_ROS_JOINTS = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']


class PolicyBridge(Node):
    def __init__(self):
        super().__init__('policy_bridge')

        str_arr = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        dbl_arr = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)

        # -----------------------------
        # Parameters (declare FIRST)
        # -----------------------------
        self.declare_parameter('controller_topic', '/so_100_arm_controller/joint_trajectory')
        self.declare_parameter('policy_url', 'http://127.0.0.1:8009/plan_joints')
        self.declare_parameter('command_dt', 0.5)

        self.declare_parameter('ros_joint_names', DEFAULT_ROS_JOINTS, descriptor=str_arr)

        # Joint limits
        self.declare_parameter('joint_min', [-3.14, -1.57, -2.50, -2.50, -2.80], descriptor=dbl_arr)
        self.declare_parameter('joint_max', [ 3.14,  1.57,  2.50,  2.50,  2.80], descriptor=dbl_arr)

        self.declare_parameter('max_step_rad', 0.30)
        self.declare_parameter('delta_scale', 1.0)
        self.declare_parameter('force_absolute_mode', False)

        # Startup smoothing
        self.declare_parameter('startup_hold_ticks', 2)
        self.declare_parameter('startup_ramp_seconds', 3.0)
        self.declare_parameter('startup_warmup_max_step_rad', 0.05)

        # Images
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('image_wrist_topic', '/camera2/color/image_raw')
        self.declare_parameter('image_encoding', 'rgb8')
        self.declare_parameter('enable_images', True)

        # Fake policy
        self.declare_parameter('use_fake_policy', False)

        # Gripper topic
        self.declare_parameter('gripper_topic', '/so_100_arm_gripper_controller/commands')

        # Enable/Disable gating
        self.declare_parameter('enable_topic', '/policy_bridge/enable')
        self.declare_parameter('disable_topic', '/policy_bridge/disable')   # ✅ NEW
        self.declare_parameter('start_enabled', False)

        # Optional: what to do on disable
        # - "hold": publish one hold-position trajectory once when disabling
        # - "none": do nothing (just stop publishing)
        self.declare_parameter('on_disable_behavior', 'hold')               # ✅ NEW

        # -----------------------------
        # Read parameters
        # -----------------------------
        gp = lambda k: self.get_parameter(k).get_parameter_value()

        self.controller_topic = gp('controller_topic').string_value
        self.policy_url = gp('policy_url').string_value
        self.command_dt = gp('command_dt').double_value

        self.ros_joint_names = list(gp('ros_joint_names').string_array_value)
        assert len(self.ros_joint_names) == 5, f"Expected 5 arm joints, got {len(self.ros_joint_names)}: {self.ros_joint_names}"
        assert self.ros_joint_names == DEFAULT_ROS_JOINTS, f"Joint order mismatch! Expected {DEFAULT_ROS_JOINTS}, got {self.ros_joint_names}"

        self.qmin = np.array(list(gp('joint_min').double_array_value), dtype=np.float64)
        self.qmax = np.array(list(gp('joint_max').double_array_value), dtype=np.float64)
        assert len(self.qmin) == len(self.ros_joint_names), f"joint_min length ({len(self.qmin)}) != joint count ({len(self.ros_joint_names)})"
        assert len(self.qmax) == len(self.ros_joint_names), f"joint_max length ({len(self.qmax)}) != joint count ({len(self.ros_joint_names)})"

        self.max_step = gp('max_step_rad').double_value
        self.delta_scale = gp('delta_scale').double_value
        self.force_absolute = gp('force_absolute_mode').bool_value

        self.startup_hold_ticks = int(gp('startup_hold_ticks').integer_value)
        self.startup_ramp_seconds = gp('startup_ramp_seconds').double_value
        self.startup_warmup_max_step = gp('startup_warmup_max_step_rad').double_value

        self.enable_images = gp('enable_images').bool_value
        self.image_topic = gp('image_topic').string_value
        self.image_wrist_topic = gp('image_wrist_topic').string_value
        self.image_encoding = gp('image_encoding').string_value

        self.use_fake_policy = gp('use_fake_policy').bool_value
        self.gripper_topic = gp('gripper_topic').string_value

        # Enable/Disable gating params
        self.enable_topic = gp('enable_topic').string_value
        self.disable_topic = gp('disable_topic').string_value
        self.enabled = gp('start_enabled').bool_value
        self.on_disable_behavior = gp('on_disable_behavior').string_value.lower().strip()

        # -----------------------------
        # State
        # -----------------------------
        self.safety = Safety(self.qmin, self.qmax, self.max_step, js_timeout=1.5)
        self.q_curr = None
        self.last_js_names = []

        self._startup_tick_count = 0
        self._startup_t0 = self.get_clock().now()
        self._last_q_commanded = None
        self._last_q_actual = None

        # If we want to publish a one-shot "hold" when disabling
        self._pending_disable_hold_publish = False

        # Images
        self.last_image = None
        self.last_wrist_image = None
        self._scene_image_received = False
        self._wrist_image_received = False

        # -----------------------------
        # cv_bridge init (optional)
        # -----------------------------
        if CV_BRIDGE_AVAILABLE and self.enable_images:
            try:
                self.bridge = CvBridge()
                self.get_logger().info("cv_bridge initialized successfully")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize cv_bridge: {e}. Image processing disabled.")
                self.bridge = None
                self.enable_images = False
        else:
            if not CV_BRIDGE_AVAILABLE:
                self.get_logger().warn(f"cv_bridge not available: {cv_bridge_error}. Image processing disabled.")
            self.bridge = None
            if not self.enable_images:
                self.get_logger().info("Image processing disabled by parameter")

        # -----------------------------
        # ROS pubs/subs
        # -----------------------------
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.on_js, 10)

        # Enable/Disable gate subscribers
        self.sub_enable = self.create_subscription(Bool, self.enable_topic, self.on_enable, 10)
        self.sub_disable = self.create_subscription(Bool, self.disable_topic, self.on_disable, 10)  # ✅ NEW

        self.get_logger().info(
            f"Gate topics: enable={self.enable_topic}, disable={self.disable_topic}, start_enabled={self.enabled}, "
            f"on_disable_behavior={self.on_disable_behavior}"
        )

        # Controller publishers
        self.pub_traj = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.pub_gripper = self.create_publisher(Float64MultiArray, self.gripper_topic, 10)
        self.get_logger().info(f"Gripper control publisher: {self.gripper_topic}")

        # Image subs with best-effort QoS
        if self.enable_images:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
            self.image_wrist_sub = self.create_subscription(Image, self.image_wrist_topic, self.on_wrist_image, qos)
            self.get_logger().info(f"Subscribed to scene camera: {self.image_topic}")
            self.get_logger().info(f"Subscribed to wrist camera: {self.image_wrist_topic}")

        # Timer tick
        self.timer = self.create_timer(self.command_dt, self.tick)

        if self.use_fake_policy:
            self.get_logger().info("[FAKE POLICY] Enabled - will use q_cmd = q_curr + [0.1, 0, 0, 0, 0] instead of HTTP request")
        else:
            self.get_logger().info(f"PolicyBridge: {self.policy_url} → {self.controller_topic}")
        self.get_logger().info(f"Subscribed to /joint_states, expecting joints: {self.ros_joint_names}")

    # -----------------------------
    # Enable/Disable gate callbacks
    # -----------------------------
    def on_enable(self, msg: Bool):
        new_enabled = bool(msg.data)
        if new_enabled and not self.enabled:
            # Transition: disabled -> enabled. Reset startup smoothing/ramp timers.
            self._startup_tick_count = 0
            self._startup_t0 = self.get_clock().now()
            if hasattr(self, '_post_startup_logged'):
                delattr(self, '_post_startup_logged')
            self.get_logger().info("PolicyBridge ENABLED: resetting startup smoothing timers")
        self.enabled = new_enabled
        self.get_logger().info(f"PolicyBridge enabled={self.enabled}")

    def on_disable(self, msg: Bool):
        """
        If msg.data == True -> DISABLE immediately
        If msg.data == False -> no-op (you can re-enable via /policy_bridge/enable)
        """
        if not bool(msg.data):
            return

        if self.enabled:
            self.enabled = False
            self.get_logger().info("PolicyBridge DISABLED via disable_topic")

            # Optionally publish a one-shot hold command at the current q_curr
            if self.on_disable_behavior == 'hold':
                self._pending_disable_hold_publish = True

    # -----------------------------
    # Joint state callback
    # -----------------------------
    def on_js(self, msg: JointState):
        self.last_js_names = list(msg.name)
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            self.q_curr = np.array([name_to_pos[n] for n in self.ros_joint_names], dtype=np.float64)
            assert len(self.q_curr) == len(self.ros_joint_names), \
                f"q_curr length ({len(self.q_curr)}) != ros_joint_names length ({len(self.ros_joint_names)})"
            self.safety.mark_js()

            if not hasattr(self, '_js_received_logged'):
                self.get_logger().info(f"Joint states mapped: {len(self.q_curr)} joints, order: {self.ros_joint_names}")
                self._js_received_logged = True
                self._js_update_count = 0

            self._js_update_count = getattr(self, '_js_update_count', 0) + 1
            if self._js_update_count % 100 == 0:
                self.get_logger().info(f"Joint states updated {self._js_update_count} times, current: {self.q_curr}")
        except KeyError:
            missing = [n for n in self.ros_joint_names if n not in name_to_pos]
            self.get_logger().error(f"❌ Missing joints in /joint_states: {missing}")
            self.get_logger().error(f"  Available joints: {self.last_js_names}")
            self.get_logger().error(f"  Expected joints: {self.ros_joint_names}")
            raise

    # -----------------------------
    # Image callbacks
    # -----------------------------
    def on_image(self, msg: Image):
        if self.bridge is None or not self.enable_images:
            return
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)
            if not self._scene_image_received:
                self.get_logger().info(f"Scene camera image received: {self.last_image.shape}")
                self._scene_image_received = True
                self._scene_image_count = 0
            self._scene_image_count = getattr(self, '_scene_image_count', 0) + 1
            if self._scene_image_count % 150 == 0:
                self.get_logger().info(f"Scene camera: {self._scene_image_count} images received")
        except Exception as e:
            self.get_logger().debug(f"Scene image convert failed: {e}")

    def on_wrist_image(self, msg: Image):
        if self.bridge is None or not self.enable_images:
            return
        try:
            self.last_wrist_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)
            if not self._wrist_image_received:
                self.get_logger().info(f"Wrist camera image received: {self.last_wrist_image.shape}")
                self._wrist_image_received = True
                self._wrist_image_count = 0
            self._wrist_image_count = getattr(self, '_wrist_image_count', 0) + 1
            if self._wrist_image_count % 150 == 0:
                self.get_logger().info(f"Wrist camera: {self._wrist_image_count} images received")
        except Exception as e:
            self.get_logger().debug(f"Wrist image convert failed: {e}")

    # -----------------------------
    # Helper: one-shot hold publish
    # -----------------------------
    def _publish_hold_once(self):
        if self.q_curr is None:
            return
        traj = JointTrajectory()
        traj.joint_names = self.ros_joint_names
        pt = JointTrajectoryPoint()
        pt.positions = self.q_curr.tolist()
        pt.time_from_start.sec = int(self.command_dt)
        pt.time_from_start.nanosec = int((self.command_dt - int(self.command_dt)) * 1e9)
        traj.points = [pt]
        self.pub_traj.publish(traj)
        self.get_logger().info("Published one-shot HOLD trajectory on disable")

    # -----------------------------
    # Main tick loop
    # -----------------------------
    def tick(self):
        # If we just disabled and want a one-shot hold, do it and clear the flag.
        if self._pending_disable_hold_publish:
            self._pending_disable_hold_publish = False
            self._publish_hold_once()

        # Gate: no publishing + no HTTP while disabled
        if not self.enabled:
            if not hasattr(self, '_disabled_log_count'):
                self._disabled_log_count = 0
            self._disabled_log_count += 1
            if self._disabled_log_count % 50 == 1:
                self.get_logger().info("PolicyBridge is DISABLED (waiting for enable=true)")
            return

        if self.q_curr is None:
            self.get_logger().warn("Waiting for /joint_states...")
            return

        if not self.safety.ok():
            if not hasattr(self, '_safety_hold_log_count'):
                self._safety_hold_log_count = 0
            self._safety_hold_log_count += 1
            if self._safety_hold_log_count % 50 == 0:
                self.get_logger().warn(f"Safety hold (no joint_states or estop) - {self._safety_hold_log_count} times")
            return

        try:
            # Startup hold: hold current pose for N ticks after enabling
            if self._startup_tick_count < self.startup_hold_ticks:
                self._startup_tick_count += 1
                traj = JointTrajectory()
                traj.joint_names = self.ros_joint_names
                pt = JointTrajectoryPoint()
                pt.positions = self.q_curr.tolist()
                pt.time_from_start.sec = int(self.command_dt)
                pt.time_from_start.nanosec = int((self.command_dt - int(self.command_dt)) * 1e9)
                traj.points = [pt]
                self.pub_traj.publish(traj)

                if self._startup_tick_count == 1:
                    self.get_logger().info(
                        f"Startup smoothing: holding current pose for {self.startup_hold_ticks} ticks "
                        f"(tick {self._startup_tick_count}/{self.startup_hold_ticks})"
                    )
                    self.get_logger().info(f"  Current joints: {self.q_curr}")
                    self.get_logger().info(
                        f"  Scene camera: {'✓' if self._scene_image_received else '✗'}, "
                        f"Wrist camera: {'✓' if self._wrist_image_received else '✗'}"
                    )
                return

            if not hasattr(self, '_post_startup_logged'):
                self.get_logger().info("✓ Startup complete! Starting policy ticks.")
                self._post_startup_logged = True
                self._tick_count = 0

            # Startup ramp: step limit ramps up from warmup -> max_step
            elapsed = (self.get_clock().now() - self._startup_t0).nanoseconds * 1e-9
            if self.startup_ramp_seconds > 0.0 and elapsed < self.startup_ramp_seconds:
                alpha = max(0.0, min(1.0, elapsed / self.startup_ramp_seconds))
                ramp_step = (1.0 - alpha) * self.startup_warmup_max_step + alpha * self.max_step
                self.safety.max_step = float(ramp_step)
            else:
                self.safety.max_step = float(self.max_step)

            # -----------------------------
            # Policy -> delta vec
            # -----------------------------
            if self.use_fake_policy:
                vec = np.array([0.1, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
                mode = 'delta'
                gripper_value = None
            else:
                # Send last commanded (clamped) position if available, else actual
                q_for_model = self._last_q_commanded.copy() if self._last_q_commanded is not None else self.q_curr.copy()
                payload = {'joints': q_for_model.tolist()}

                # Attach images (optional)
                if self.enable_images and self.bridge is not None and CV_BRIDGE_AVAILABLE:
                    target_pixels = 640 * 480
                    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]

                    if self.last_image is not None:
                        scene_pixels = self.last_image.shape[0] * self.last_image.shape[1]
                        small_scene = self.last_image[::2, ::2, :] if scene_pixels > target_pixels else self.last_image
                        scene_bgr = cv2.cvtColor(small_scene, cv2.COLOR_RGB2BGR)
                        ok_scene, buf_scene = cv2.imencode(".jpg", scene_bgr, encode_params)
                        if ok_scene:
                            payload['image_scene_jpeg'] = buf_scene.tobytes().hex()

                            if self.last_wrist_image is not None:
                                wrist_pixels = self.last_wrist_image.shape[0] * self.last_wrist_image.shape[1]
                                small_wrist = self.last_wrist_image[::2, ::2, :] if wrist_pixels > target_pixels else self.last_wrist_image
                                wrist_bgr = cv2.cvtColor(small_wrist, cv2.COLOR_RGB2BGR)
                                ok_wrist, buf_wrist = cv2.imencode(".jpg", wrist_bgr, encode_params)
                                if ok_wrist:
                                    payload['image_wrist_jpeg'] = buf_wrist.tobytes().hex()

                r = requests.post(self.policy_url, json=payload, timeout=0.5)
                r.raise_for_status()
                out = r.json()
                mode = out.get('mode', 'delta')
                joints_output = np.array(out['joints'], dtype=np.float64)

                # Gripper (optional)
                gripper_value = None
                if 'gripper' in out:
                    gripper_value = float(out['gripper'])
                elif len(joints_output) > len(self.ros_joint_names):
                    gripper_value = float(joints_output[len(self.ros_joint_names)])

                vec = joints_output[:len(self.ros_joint_names)]
                assert len(vec) == len(self.ros_joint_names), "Policy response joint length mismatch"

            # Scale deltas
            vec = vec * self.delta_scale

            # -----------------------------
            # Delta/Abs -> target
            # -----------------------------
            if mode == 'abs':
                q_target = vec
            else:
                q_target = self.q_curr + vec
                q_target = self.safety.step_limit(self.q_curr, q_target)
                q_target = self.safety.joint_limits(q_target)

            # Publish trajectory
            traj = JointTrajectory()
            traj.joint_names = self.ros_joint_names
            pt = JointTrajectoryPoint()
            pt.positions = q_target.tolist()
            pt.time_from_start.sec = int(self.command_dt)
            pt.time_from_start.nanosec = int((self.command_dt - int(self.command_dt)) * 1e9)
            traj.points = [pt]
            self.pub_traj.publish(traj)

            # Remember last commanded
            self._last_q_commanded = q_target.copy()
            self._last_q_actual = self.q_curr.copy()

            # Publish gripper if present
            if gripper_value is not None:
                gripper_cmd = Float64MultiArray()
                gripper_cmd.data = [float(np.clip(gripper_value, -1.0, 1.0))]
                self.pub_gripper.publish(gripper_cmd)

        except Exception as e:
            import traceback
            self.get_logger().error(f"Policy tick failed: {e}\n{traceback.format_exc()}")


def main():
    rclpy.init()
    node = PolicyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
