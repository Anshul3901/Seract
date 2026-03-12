#!/usr/bin/env python3
"""
Async (non-blocking) viewpoint runner with anti-blur capture + AI policy bridge transition.

Behavior:
1) Wait for image stream + arm server.
2) (Optional) compute drift compensation from /joint_states at startup (assumes robot starts at HOME).
3) Go to start pose FIRST.
4) ONLY then look for object at START pose (optional gating, unlimited retries).
5) Capture at start after settle + stable frames.
6) Move to each recorded viewpoint pose and capture after settle + stable frames.
7) After finishing viewpoints, call DecisionAct action server to wait for decision.json to be populated
   and run the policy flow. When the action returns, this node loops and repeats.

Update:
- Added an intermediate waypoint between RIGHT and FRONT to reduce the arm "cocking back".
- Added support for multi-point trajectories so special transitions can be shaped explicitly.
"""

import os
import csv
import json
import time
import shutil
from datetime import datetime
from typing import Optional, Tuple, List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import cv2

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from so101_motion_interfaces.action import DecisionAct


DEFAULT_JOINTS = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']


class BBoxViewpointDriver(Node):
    def __init__(self):
        super().__init__('bbox_viewpoint_driver')

        # ---------------- Params ----------------
        self.declare_parameter('image_topic', '/camera2/color/image_raw')
        self.declare_parameter('bbox_topic', '/object_bbox')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('arm_action', '/so_100_arm_controller/follow_joint_trajectory')

        self.declare_parameter('save_dir', os.path.expanduser('~/seract_orch_ws/scanned_pictures'))
        dir_path = self.get_parameter('save_dir').value
        shutil.rmtree(dir_path, ignore_errors=True)
        os.makedirs(dir_path, exist_ok=True)

        self.declare_parameter('move_duration', 2.0)

        # Start-only gating
        self.declare_parameter('require_seen_at_start', True)
        self.declare_parameter('start_seen_timeout_sec', 3.0)  # kept but unused for abort (unlimited retries)

        # BBox validity (used ONLY for start gating + overlay text)
        self.declare_parameter('min_area_ratio', 0.01)
        self.declare_parameter('max_area_ratio', 0.80)
        self.declare_parameter('border_margin_px', 12)
        self.declare_parameter('min_bbox_w_px', 30.0)
        self.declare_parameter('min_bbox_h_px', 30.0)

        self.declare_parameter('stable_frames', 6)
        self.declare_parameter('stable_std_px', 8.0)
        self.declare_parameter('seen_required_frames', 6)

        # --- Expected bbox ---
        self.declare_parameter('use_expected_bbox', False)
        self.declare_parameter('expected_cx', 306.5)
        self.declare_parameter('expected_cy', 223.5)
        self.declare_parameter('expected_w', 401.0)
        self.declare_parameter('expected_h', 425.0)
        self.declare_parameter('cx_tol', 70.0)
        self.declare_parameter('cy_tol', 70.0)
        self.declare_parameter('w_tol', 120.0)
        self.declare_parameter('h_tol', 120.0)

        # Optional auto-capture on "seen"
        self.declare_parameter('auto_capture_on_seen', False)
        self.declare_parameter('auto_capture_cooldown_sec', 2.0)

        # Anti-blur / stabilization
        self.declare_parameter('settle_time_sec', 0.6)
        self.declare_parameter('stable_image_frames', 5)
        self.declare_parameter('stable_image_threshold', 2.0)
        self.declare_parameter('max_stable_wait_sec', 2.5)
        self.declare_parameter('save_as_jpeg', False)
        self.declare_parameter('capture_jpeg_quality', 95)

        # Policy bridge transition (kept)
        self.declare_parameter('policy_enable_topic', '/policy_bridge/enable')
        self.declare_parameter('policy_enable_delay_sec', 12.0)
        self.declare_parameter('policy_disable_delay_sec', 12.0)

        # Decision movement (now means: call action server)
        self.declare_parameter('run_decision_movement', True)

        # Optional delay between cycles
        self.declare_parameter('cycle_pause_sec', 0.5)

        # ---------------- Drift compensation ----------------
        self.declare_parameter('enable_drift_comp', False)

        # HOME reference ("was")
        self.declare_parameter('home_ref_was', [
            0.7946110450795297,
            -0.5825958188153311,
            -1.1523718255869668,
            -1.5124784853700515,
            -0.8124885215794307
        ])

        # Safety clamp for drift offset per joint (radians)
        self.declare_parameter('max_abs_drift_comp', 3.2)

        # ---------------- Joint limits ----------------
        self.joint_limits = {
            'Shoulder_Rotation': {'min': -3.14, 'max': 3.14},
            'Shoulder_Pitch':    {'min': -1.6,  'max': 2.57},
            'Elbow':             {'min': -2.5,  'max': -0.5},
            'Wrist_Pitch':       {'min': -2.5,  'max': 2.5},
            'Wrist_Roll':        {'min': -2.8,  'max': 2.8}
        }
        self.limit_margin = 0.03

        # ---------------- Recorded joint poses ----------------
        self.start_q = np.array([
            1.5754027676359372,
            -1.0680923344947737,
            -0.78941063727839,
            -1.5576592082616179,
            1.9079540621012334
        ], dtype=float)

        self.views = {
            "top": np.array([
                1.5823124288974983,
                -0.7152526132404181,
                -1.2685673215141353,
                -1.2177280550774525,
                1.8519778817524455
            ], dtype=float),

            "left": np.array([
                1.022629866711047,
                -1.4510191637630663,
                -0.785816962146622,
                -1.5877796901893286,
                2.7094853253934494
            ], dtype=float),

            "right": np.array([
                2.158117534027592,
                -0.9244947735191639,
                -1.3931480594154289,
                -1.2650602409638554,
                -1.4938475665748392
            ], dtype=float),

            # New intermediate waypoint to reduce cock-back on right -> front
            "right_front_mid": np.array([
                1.85,
                -0.15,
                -1.05,
                -1.18,
                1.10
            ], dtype=float),

            "front": np.array([
                1.5684931063743761,
                0.6635694729637235,
                -0.7067561092477239,
                -1.0993975903614457,
                1.850786899191833
            ], dtype=float),

            "back": np.array([
                1.5707963267948966,
                -0.5292595818815331,
                -1.595591758505031,
                -1.0993975903614457,
                1.850786899191833
            ], dtype=float),

            # Pre-decision position: move here before calling action server
            "pre_decision": np.array([
                1.6491058210925893,
                -0.12581881533101044,
                -2.166986104456157,
                -0.6820137693631669,
                1.6018715440238196
            ], dtype=float),
        }

        # Keep captures only for the real viewpoints, not the internal transition waypoint
        # Note: "top" view is captured at start position when object is detected
        self.view_order = ["left", "right", "front", "back"]

        # Explicit transition map: current_view -> next_view -> list of intermediate waypoints
        self.transition_waypoints = {
            ("right", "front"): ["right_front_mid"]
        }

        # ---------------- State ----------------
        self.bridge = CvBridge()
        self.last_image: Optional[np.ndarray] = None
        self.img_w: Optional[int] = None
        self.img_h: Optional[int] = None

        self.last_bbox: Optional[Dict[str, float]] = None
        self.bbox_center_hist: List[Tuple[float, float]] = []
        self.valid_hist: List[bool] = []
        self.last_auto_capture_t = 0.0

        # Joint state tracking for drift compensation
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.latest_joint_state_map: Dict[str, float] = {}
        self.drift_offset = np.zeros(len(self.joint_names), dtype=float)
        self.drift_ready = False

        # Async motion bookkeeping
        self.pending_action = False
        self.pending_action_name = ""
        self.view_index = 0
        self.start_seen_t0 = None

        # Track last captured real view to shape transitions
        self.last_completed_view: Optional[str] = None
        self.current_move_target_name: Optional[str] = None

        # Anti-blur stability tracking
        self.prev_gray = None
        self.stable_count = 0
        self.capture_ready_t0 = None
        self.wait_after_move_until = 0.0

        # Policy bridge state (kept)
        self.policy_pub = self.create_publisher(Bool, self.get_parameter('policy_enable_topic').value, 10)
        self.policy_t0 = 0.0

        # Decision Action client state
        self.decision_client = ActionClient(self, DecisionAct, "/decision_movement/run")
        self.decision_goal_sent = False
        self.decision_done = False
        self.decision_result: Optional[DecisionAct.Result] = None

        # ---------------- Cycle Logger ----------------
        self.cycle_log_dir = os.path.expanduser('~/seract_orch_ws/cycle_logs')
        os.makedirs(self.cycle_log_dir, exist_ok=True)
        self.cycle_log_path = os.path.join(self.cycle_log_dir, 'cycle_log.csv')
        self._init_cycle_csv()

        # Loop counter – resume from last row in existing CSV
        self.cycle_count = self._read_last_cycle_number()

        # Timing stamps
        self.t_cycle_start: Optional[float] = None
        self.t_scanning_done: Optional[float] = None
        self.t_decision_sent: Optional[float] = None
        self.t_decision_done: Optional[float] = None

        # Vision result tracking
        self.latest_vision_item_id: Optional[str] = None
        self.latest_vision_object_type: Optional[str] = None
        self.latest_vision_decision: Optional[str] = None

        # ---------------- ROS IO ----------------
        image_topic = self.get_parameter('image_topic').value
        bbox_topic = self.get_parameter('bbox_topic').value
        js_topic = self.get_parameter('joint_states_topic').value
        arm_action = self.get_parameter('arm_action').value

        for jn in self.joint_names:
            if jn not in self.joint_limits:
                self.get_logger().warn(f"Joint '{jn}' not found in joint_limits; it will not be clamped.")

        self.create_subscription(Image, image_topic, self.on_image, 10)
        self.create_subscription(Detection2DArray, bbox_topic, self.on_bbox, 10)
        self.create_subscription(JointState, js_topic, self.on_joint_state, 10)
        self.create_subscription(String, '/vision_detection_results', self._on_vision_result, 10)

        self.arm_client = ActionClient(self, FollowJointTrajectory, arm_action)

        self.get_logger().info(f"Sub image: {image_topic}")
        self.get_logger().info(f"Sub bbox:  {bbox_topic}")
        self.get_logger().info(f"Sub joints: {js_topic}")
        self.get_logger().info(f"Arm action: {arm_action}")
        self.get_logger().info(f"Policy enable pub: {self.get_parameter('policy_enable_topic').value}")
        self.get_logger().info("Decision action client: decision_movement/run")
        self.get_logger().info(f"📊 Cycle log: {self.cycle_log_path}")

        # Clamp recorded poses once at startup
        self.start_q = self.clamp_joints(self.start_q)
        for k in list(self.views.keys()):
            self.views[k] = self.clamp_joints(self.views[k])

        # State machine
        self.state = "WAIT_INPUTS"
        self._last_wait_log_t = 0.0

        self.create_timer(0.1, self.tick)

    # ---------------- Loop Reset ----------------
    def _reset_for_next_cycle(self):
        self.cycle_count += 1

        self._write_cycle_log()

        pause = float(self.get_parameter('cycle_pause_sec').value)

        self.get_logger().warn(f"🔁 Resetting for next cycle (cycle={self.cycle_count})... pause={pause:.2f}s")

        if pause > 0.0:
            time.sleep(pause)

        self.view_index = 0
        self.last_completed_view = None
        self.current_move_target_name = None

        self.bbox_center_hist.clear()
        self.valid_hist.clear()
        self.last_bbox = None

        self.prev_gray = None
        self.stable_count = 0
        self.capture_ready_t0 = None
        self.wait_after_move_until = 0.0

        self.decision_goal_sent = False
        self.decision_done = False
        self.decision_result = None

        self.pending_action = False
        self.pending_action_name = ""

        self._reset_cycle_timing()

        self.state = "MOVE_START"

    # ---------------- Callbacks ----------------
    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_image = bgr
            self.img_h, self.img_w = bgr.shape[:2]
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")

    def on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_state_map[name] = float(pos)

    def on_bbox(self, msg: Detection2DArray):
        if not msg.detections:
            self._push_valid(False)
            self.last_bbox = None
            return

        det = msg.detections[0]
        bb = det.bbox
        cx = float(bb.center.position.x)
        cy = float(bb.center.position.y)
        w = float(bb.size_x)
        h = float(bb.size_y)

        self.last_bbox = {"cx": cx, "cy": cy, "w": w, "h": h}
        self.bbox_center_hist.append((cx, cy))
        if len(self.bbox_center_hist) > 50:
            self.bbox_center_hist.pop(0)

        is_valid = self.is_bbox_valid(cx, cy, w, h)
        self._push_valid(is_valid)

        if self.get_parameter('auto_capture_on_seen').value and self.object_seen():
            now = time.time()
            cooldown = float(self.get_parameter('auto_capture_cooldown_sec').value)
            if (now - self.last_auto_capture_t) >= cooldown:
                self.last_auto_capture_t = now
                self.save_capture("auto_seen")

    def _push_valid(self, v: bool):
        self.valid_hist.append(bool(v))
        if len(self.valid_hist) > 50:
            self.valid_hist.pop(0)

    # ---------------- Vision result callback ----------------
    def _on_vision_result(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_vision_item_id = data.get('item_id', None)
            self.latest_vision_decision = data.get('result', None)

            obj_id = data.get('object_identification', None)
            if obj_id and isinstance(obj_id, dict):
                self.latest_vision_object_type = obj_id.get('object_type', None)
            elif obj_id and isinstance(obj_id, str):
                self.latest_vision_object_type = obj_id
            else:
                self.latest_vision_object_type = None

            self.get_logger().info(
                f"📊 Vision result received: item={self.latest_vision_item_id} "
                f"object={self.latest_vision_object_type} decision={self.latest_vision_decision}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to parse vision result: {e}")

    # ---------------- Cycle CSV Logger ----------------
    CYCLE_CSV_FIELDS = [
        'cycle', 'item_id', 'object_type', 'decision',
        'total_cycle_sec', 'scanning_sec', 'policy_sec',
        'timestamp',
    ]

    def _init_cycle_csv(self):
        if not os.path.exists(self.cycle_log_path):
            with open(self.cycle_log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.CYCLE_CSV_FIELDS)
            self.get_logger().info(f"📊 Created cycle log: {self.cycle_log_path}")

    def _read_last_cycle_number(self) -> int:
        try:
            if not os.path.exists(self.cycle_log_path):
                return 0
            with open(self.cycle_log_path, 'r', newline='') as f:
                reader = csv.reader(f)
                last_cycle = 0
                for row in reader:
                    if not row or row[0] == 'cycle':
                        continue
                    try:
                        last_cycle = int(row[0])
                    except (ValueError, IndexError):
                        continue
                self.get_logger().info(f"📊 Resuming from cycle {last_cycle} (found in existing CSV)")
                return last_cycle
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not read last cycle number from CSV: {e}")
            return 0

    def _write_cycle_log(self):
        try:
            total_sec = ''
            scanning_sec = ''
            policy_sec = ''

            if self.t_cycle_start is not None and self.t_decision_done is not None:
                total_sec = f"{self.t_decision_done - self.t_cycle_start:.1f}"
            elif self.t_cycle_start is not None:
                total_sec = f"{time.time() - self.t_cycle_start:.1f}"

            if self.t_cycle_start is not None and self.t_scanning_done is not None:
                scanning_sec = f"{self.t_scanning_done - self.t_cycle_start:.1f}"

            if self.t_decision_sent is not None and self.t_decision_done is not None:
                policy_sec = f"{self.t_decision_done - self.t_decision_sent:.1f}"

            decision = ''
            if self.decision_result is not None and getattr(self.decision_result, 'decision', ''):
                decision = self.decision_result.decision
            elif self.latest_vision_decision:
                decision = self.latest_vision_decision

            row = [
                self.cycle_count,
                self.latest_vision_item_id or '',
                self.latest_vision_object_type or '',
                decision,
                total_sec,
                scanning_sec,
                policy_sec,
                datetime.now().isoformat(timespec='seconds'),
            ]

            with open(self.cycle_log_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)

            self.get_logger().info(
                f"📊 Cycle {self.cycle_count} logged → "
                f"total={total_sec}s  scan={scanning_sec}s  policy={policy_sec}s  "
                f"decision={decision}  object={self.latest_vision_object_type or '?'}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Failed to write cycle log: {e}")

    def _reset_cycle_timing(self):
        self.t_cycle_start = None
        self.t_scanning_done = None
        self.t_decision_sent = None
        self.t_decision_done = None
        self.latest_vision_item_id = None
        self.latest_vision_object_type = None
        self.latest_vision_decision = None

    # ---------------- Drift compensation ----------------
    def compute_drift_offset_if_ready(self) -> bool:
        if not bool(self.get_parameter('enable_drift_comp').value):
            self.drift_offset[:] = 0.0
            self.drift_ready = True
            return True

        missing = [jn for jn in self.joint_names if jn not in self.latest_joint_state_map]
        if missing:
            return False

        current = np.array([self.latest_joint_state_map[jn] for jn in self.joint_names], dtype=float)
        home_ref = np.array(self.get_parameter('home_ref_was').value, dtype=float)

        if home_ref.shape[0] != current.shape[0]:
            self.get_logger().error(
                f"home_ref_was length {home_ref.shape[0]} != joint_names length {current.shape[0]}"
            )
            return False

        drift = current - home_ref
        max_abs = float(self.get_parameter('max_abs_drift_comp').value)
        drift = np.clip(drift, -max_abs, max_abs)

        self.drift_offset = drift
        self.drift_ready = True

        self.get_logger().info(
            "Drift compensation enabled.\n"
            f"  home_ref_was: {np.array2string(home_ref, precision=3)}\n"
            f"  home_current: {np.array2string(current, precision=3)}\n"
            f"  drift_offset: {np.array2string(self.drift_offset, precision=3)}\n"
            "  Applying q_cmd = q_desired - drift_offset"
        )
        return True

    def apply_drift_comp(self, q: np.ndarray) -> np.ndarray:
        if not self.drift_ready:
            return np.array(q, dtype=float)
        return np.array(q, dtype=float) - self.drift_offset

    # ---------------- Joint limit utilities ----------------
    def clamp_joints(self, q: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=float).copy()
        for i, jn in enumerate(self.joint_names):
            lim = self.joint_limits.get(jn, None)
            if lim is None:
                continue
            jmin = lim['min'] + self.limit_margin
            jmax = lim['max'] - self.limit_margin
            q[i] = float(np.clip(q[i], jmin, jmax))
        return q

    # ---------------- Trajectory helpers ----------------
    def build_transition_sequence(self, current_view: Optional[str], target_view: str) -> List[np.ndarray]:
        """
        Returns the full joint sequence to send for a move.
        Includes intermediate waypoints for selected problematic transitions.
        """
        seq: List[np.ndarray] = []

        if current_view is not None:
            key = (current_view, target_view)
            if key in self.transition_waypoints:
                for wp_name in self.transition_waypoints[key]:
                    if wp_name not in self.views:
                        self.get_logger().warn(f"Transition waypoint '{wp_name}' not found in self.views")
                        continue
                    seq.append(self.views[wp_name])

        seq.append(self.views[target_view])
        return seq

    # ---------------- Async action send (arm) ----------------
    def send_joints_async(self, q: np.ndarray, duration: float, action_name: str):
        self.send_trajectory_async([q], duration, action_name)

    def send_trajectory_async(self, q_list: List[np.ndarray], total_duration: float, action_name: str):
        if self.pending_action:
            self.get_logger().warn(f"Action already pending ({self.pending_action_name}), skipping send {action_name}")
            return

        if not q_list:
            self.get_logger().warn(f"[{action_name}] Empty trajectory requested, skipping.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        n_pts = len(q_list)
        for idx, q in enumerate(q_list):
            q_cmd = self.apply_drift_comp(np.array(q, dtype=float))
            q_cmd = self.clamp_joints(q_cmd)

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q_cmd.tolist()]

            # Evenly distribute points through total_duration
            t = total_duration * float(idx + 1) / float(n_pts)
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start.sec = sec
            pt.time_from_start.nanosec = nsec
            goal.trajectory.points.append(pt)

        self.pending_action = True
        self.pending_action_name = action_name

        self.get_logger().info(
            f"[{action_name}] Sending trajectory with {len(goal.trajectory.points)} point(s)."
        )

        fut = self.arm_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error(f"[{self.pending_action_name}] Goal response None (server down?)")
            self.pending_action = False
            self.state = "WAIT_INPUTS"
            return

        if not goal_handle.accepted:
            self.get_logger().error(f"[{self.pending_action_name}] Goal rejected. Check joint_names/order.")
            self.pending_action = False
            self.state = "WAIT_INPUTS"
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def reset_stability(self):
        self.prev_gray = None
        self.stable_count = 0
        self.capture_ready_t0 = time.time()

    def _on_result(self, future):
        res = future.result().result
        err_code = getattr(res, "error_code", 0)
        err_str = getattr(res, "error_string", "")

        if int(err_code) != 0:
            self.get_logger().error(f"[{self.pending_action_name}] Exec failed: error_code={err_code} error='{err_str}'")
            self.pending_action = False
            self.state = "WAIT_INPUTS"
            return

        self.get_logger().info(f"[{self.pending_action_name}] Exec success")
        self.pending_action = False

        settle = float(self.get_parameter('settle_time_sec').value)
        self.wait_after_move_until = time.time() + settle
        self.reset_stability()

        if self.state == "WAIT_START_RESULT":
            self.state = "WAIT_START_SEEN" if bool(self.get_parameter('require_seen_at_start').value) else "CAPTURE_START"
            self.start_seen_t0 = time.time()
        elif self.state == "WAIT_VIEW_RESULT":
            self.state = "CAPTURE_VIEW"
        elif self.state == "WAIT_PRE_DECISION_RESULT":
            self.state = "RUN_DECISION"

    # ---------------- BBox validity checks ----------------
    def bbox_stable(self) -> bool:
        n = int(self.get_parameter('stable_frames').value)
        std_px = float(self.get_parameter('stable_std_px').value)
        if len(self.bbox_center_hist) < n:
            return False
        pts = np.array(self.bbox_center_hist[-n:], dtype=float)
        std = pts.std(axis=0)
        return (std[0] < std_px) and (std[1] < std_px)

    def is_bbox_valid(self, cx: float, cy: float, w: float, h: float) -> bool:
        if self.img_w is None or self.img_h is None:
            return False
        if w <= 0 or h <= 0:
            return False

        min_w = float(self.get_parameter('min_bbox_w_px').value)
        min_h = float(self.get_parameter('min_bbox_h_px').value)
        if w < min_w or h < min_h:
            return False

        margin = int(self.get_parameter('border_margin_px').value)
        x0 = cx - w / 2.0
        x1 = cx + w / 2.0
        y0 = cy - h / 2.0
        y1 = cy + h / 2.0
        if x0 < margin or y0 < margin or x1 > (self.img_w - margin) or y1 > (self.img_h - margin):
            return False

        area_ratio = (w * h) / float(self.img_w * self.img_h)
        if area_ratio < float(self.get_parameter('min_area_ratio').value):
            return False
        if area_ratio > float(self.get_parameter('max_area_ratio').value):
            return False

        if bool(self.get_parameter('use_expected_bbox').value):
            ecx = float(self.get_parameter('expected_cx').value)
            ecy = float(self.get_parameter('expected_cy').value)
            ew = float(self.get_parameter('expected_w').value)
            eh = float(self.get_parameter('expected_h').value)
            if abs(cx - ecx) > float(self.get_parameter('cx_tol').value):
                return False
            if abs(cy - ecy) > float(self.get_parameter('cy_tol').value):
                return False
            if abs(w - ew) > float(self.get_parameter('w_tol').value):
                return False
            if abs(h - eh) > float(self.get_parameter('h_tol').value):
                return False

        if not self.bbox_stable():
            return False

        return True

    def object_seen(self) -> bool:
        n = int(self.get_parameter('seen_required_frames').value)
        if len(self.valid_hist) < n:
            return False
        return all(self.valid_hist[-n:])

    # ---------------- Anti-blur image stability ----------------
    def update_image_stability(self) -> bool:
        if self.last_image is None:
            return False

        gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.stable_count = 0
            return False

        diff = cv2.absdiff(gray, self.prev_gray)
        mad = float(np.mean(diff))
        self.prev_gray = gray

        thresh = float(self.get_parameter('stable_image_threshold').value)
        if mad < thresh:
            self.stable_count += 1
        else:
            self.stable_count = 0

        need = int(self.get_parameter('stable_image_frames').value)
        return self.stable_count >= need

    # ---------------- Capture ----------------
    def save_capture(self, view_name: str):
        save_dir = self.get_parameter('save_dir').value
        if not save_dir:
            self.get_logger().info(f"[CAPTURE] {view_name} (saving disabled)")
            return
        if self.last_image is None:
            self.get_logger().warn("No image available to save.")
            return

        os.makedirs(save_dir, exist_ok=True)
        ts = int(time.time() * 1000)

        img = self.last_image.copy()
        if self.last_bbox is not None:
            cx, cy, w, h = self.last_bbox["cx"], self.last_bbox["cy"], self.last_bbox["w"], self.last_bbox["h"]
            x0 = int(cx - w / 2)
            x1 = int(cx + w / 2)
            y0 = int(cy - h / 2)
            y1 = int(cy + h / 2)
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.circle(img, (int(cx), int(cy)), 4, (0, 0, 255), -1)

            valid = self.is_bbox_valid(cx, cy, w, h)
            txt = f"valid={valid} seen={self.object_seen()}"
            cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        save_as_jpeg = bool(self.get_parameter('save_as_jpeg').value)
        if save_as_jpeg:
            path = os.path.join(save_dir, f"{ts}_{view_name}.jpg")
            q = int(self.get_parameter('capture_jpeg_quality').value)
            cv2.imwrite(path, img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        else:
            path = os.path.join(save_dir, f"{ts}_{view_name}.png")
            cv2.imwrite(path, img)

        self.get_logger().info(f"[CAPTURE] Saved {view_name} -> {path}")

    # ---------------- DecisionAct Action Client ----------------
    def _start_decision_action(self):
        if not bool(self.get_parameter('run_decision_movement').value):
            self.get_logger().info("run_decision_movement is False; skipping decision action.")
            self.decision_done = True
            return

        if self.decision_goal_sent:
            return

        if not self.decision_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info("Waiting for decision_movement action server...")
            return

        goal = DecisionAct.Goal()
        goal.wait_for_decision = True

        self.get_logger().info("Sending DecisionAct goal: wait_for_decision=true")
        self.decision_goal_sent = True
        self.t_decision_sent = time.time()

        send_future = self.decision_client.send_goal_async(goal, feedback_callback=self._on_decision_feedback)
        send_future.add_done_callback(self._on_decision_goal_response)

    def _on_decision_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            self.get_logger().info(f"[DecisionAct] state={fb.state} progress={fb.progress:.2f}")
        except Exception:
            self.get_logger().info("[DecisionAct] feedback received")

    def _on_decision_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DecisionAct goal rejected or goal_handle None.")
            self.decision_done = True
            self.decision_result = None
            return

        self.get_logger().info("DecisionAct goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_decision_result)

    def _on_decision_result(self, future):
        try:
            wrapped = future.result()
            res = wrapped.result
            self.get_logger().info(
                f"DecisionAct DONE: success={res.success} decision='{res.decision}' message='{res.message}'"
            )
            self.decision_result = res
        except Exception as e:
            self.get_logger().error(f"DecisionAct result callback failed: {e}")
            self.decision_result = None
        finally:
            self.t_decision_done = time.time()
            self.decision_done = True

    # ---------------- State machine tick ----------------
    def tick(self):
        now = time.time()

        def log_wait(msg):
            if now - self._last_wait_log_t > 3.0:
                self.get_logger().info(msg)
                self._last_wait_log_t = now

        if self.state == "WAIT_INPUTS":
            if self.last_image is None or self.img_w is None:
                log_wait("Waiting for image...")
                return
            self.state = "WAIT_ARM_SERVER"
            return

        if self.state == "WAIT_ARM_SERVER":
            if not self.arm_client.wait_for_server(timeout_sec=0.0):
                log_wait("Waiting for arm action server...")
                return
            self.get_logger().info("Arm action server ready.")

            if not self.drift_ready:
                if self.compute_drift_offset_if_ready():
                    self.get_logger().info("Drift offset computed.")
                else:
                    log_wait("Waiting for /joint_states to compute drift offset...")
                    return

            self.state = "MOVE_START"
            return

        if self.state == "MOVE_START":
            move_dt = float(self.get_parameter('move_duration').value)
            self.get_logger().info("Going to start position...")
            self.send_joints_async(self.start_q, move_dt, "move_start")
            self.state = "WAIT_START_RESULT"
            return

        if self.state == "WAIT_START_RESULT":
            return

        if self.state == "WAIT_START_SEEN":
            if self.object_seen() and self.last_bbox is not None:
                cx, cy, w, h = self.last_bbox["cx"], self.last_bbox["cy"], self.last_bbox["w"], self.last_bbox["h"]
                if self.is_bbox_valid(cx, cy, w, h):
                    self.get_logger().info("Object seen at start. Proceeding to capture + viewpoints.")
                    self.t_cycle_start = time.time()
                    self.state = "CAPTURE_START"
                    return

            if len(self.valid_hist) == 0:
                log_wait("Waiting for bbox messages (arm already at start)...")
            else:
                log_wait("Waiting for valid bbox at start pose (unlimited retries)...")
            return

        if self.state == "CAPTURE_START":
            if time.time() < self.wait_after_move_until:
                return

            if self.update_image_stability():
                # Capture "top" view at start position when object is detected
                self.save_capture("top")
                self.view_index = 0
                self.last_completed_view = None
                self.state = "MOVE_VIEW"
                return

            if self.capture_ready_t0 is not None and (time.time() - self.capture_ready_t0) > float(self.get_parameter('max_stable_wait_sec').value):
                self.get_logger().warn("Stable image timeout at start; continuing anyway.")
                # Capture "top" view even if timeout
                self.save_capture("top")
                self.view_index = 0
                self.last_completed_view = None
                self.state = "MOVE_VIEW"
            return

        if self.state == "MOVE_VIEW":
            if self.view_index >= len(self.view_order):
                self.t_scanning_done = time.time()
                self.state = "MOVE_PRE_DECISION"
                return

            name = self.view_order[self.view_index]
            move_dt = float(self.get_parameter('move_duration').value)

            seq = self.build_transition_sequence(self.last_completed_view, name)
            self.current_move_target_name = name

            if len(seq) > 1:
                waypoint_names = self.transition_waypoints.get((self.last_completed_view, name), [])
                self.get_logger().info(
                    f"Moving to view: {name} via intermediate waypoint(s): {waypoint_names}"
                )
                total_duration = move_dt * len(seq)
                self.send_trajectory_async(seq, total_duration, f"move_view:{self.last_completed_view}_to_{name}")
            else:
                self.get_logger().info(f"Moving to view: {name}")
                self.send_joints_async(seq[0], move_dt, f"move_view:{name}")

            self.state = "WAIT_VIEW_RESULT"
            return

        if self.state == "WAIT_VIEW_RESULT":
            return

        if self.state == "CAPTURE_VIEW":
            name = self.view_order[self.view_index]

            if time.time() < self.wait_after_move_until:
                return

            if self.update_image_stability():
                self.save_capture(name)
                self.last_completed_view = name
                self.view_index += 1
                self.state = "MOVE_VIEW"
                return

            if self.capture_ready_t0 is not None and (time.time() - self.capture_ready_t0) > float(self.get_parameter('max_stable_wait_sec').value):
                self.get_logger().warn(f"Stable image timeout at {name}; capturing anyway.")
                self.save_capture(name)
                self.last_completed_view = name
                self.view_index += 1
                self.state = "MOVE_VIEW"
            return

        if self.state == "MOVE_PRE_DECISION":
            move_dt = float(self.get_parameter('move_duration').value)
            self.get_logger().info("Moving to pre-decision position before calling action server...")
            self.send_joints_async(self.views["pre_decision"], move_dt, "move_pre_decision")
            self.state = "WAIT_PRE_DECISION_RESULT"
            return

        if self.state == "WAIT_PRE_DECISION_RESULT":
            return

        if self.state == "RUN_DECISION":
            self._start_decision_action()
            if self.decision_goal_sent:
                self.state = "WAIT_DECISION_DONE"
            return

        if self.state == "WAIT_DECISION_DONE":
            if self.decision_done:
                self.get_logger().info("Decision action completed. Looping.")
                self._reset_for_next_cycle()
            return


def main():
    rclpy.init()
    node = BBoxViewpointDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()