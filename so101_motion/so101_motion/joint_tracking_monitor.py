import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


DEFAULT_ARM_JOINTS = [
    "Shoulder_Rotation",
    "Shoulder_Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
]


@dataclass
class VectorSample:
    stamp_s: float
    names: List[str]
    values: List[float]


def _now_s(node: Node) -> float:
    n = node.get_clock().now().nanoseconds
    return float(n) * 1e-9


def _index_map(from_names: List[str], to_names: List[str]) -> Optional[List[int]]:
    """Return indices into from_names for each name in to_names, or None if missing."""
    idx = []
    for n in to_names:
        if n not in from_names:
            return None
        idx.append(from_names.index(n))
    return idx


class JointTrackingMonitor(Node):
    """
    Validates that the robot is moving correctly by measuring tracking error.

    Preferred source (best): JointTrajectoryControllerState (desired vs actual).
    Fallback: compare last commanded JointTrajectory point vs /joint_states.
    """

    def __init__(self):
        super().__init__("joint_tracking_monitor")

        self.declare_parameter("arm_joint_names", DEFAULT_ARM_JOINTS)
        self.declare_parameter("controller_state_topic", "/so_100_arm_controller/state")
        self.declare_parameter("command_topic", "/so_100_arm_controller/joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("print_rate_hz", 5.0)
        self.declare_parameter("max_abs_pos_error_rad_warn", 0.20)
        self.declare_parameter("stall_vel_eps", 0.01)
        self.declare_parameter("stall_error_rad", 0.10)

        self.arm_joint_names = list(self.get_parameter("arm_joint_names").value)
        self.controller_state_topic = str(self.get_parameter("controller_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.print_rate_hz = float(self.get_parameter("print_rate_hz").value)
        self.max_abs_pos_error_rad_warn = float(self.get_parameter("max_abs_pos_error_rad_warn").value)
        self.stall_vel_eps = float(self.get_parameter("stall_vel_eps").value)
        self.stall_error_rad = float(self.get_parameter("stall_error_rad").value)

        # Latest controller-state samples (preferred)
        self._last_desired: Optional[VectorSample] = None
        self._last_actual: Optional[VectorSample] = None
        self._last_error: Optional[VectorSample] = None
        self._last_actual_vel: Optional[VectorSample] = None

        # Fallback: last commanded point and latest joint_states
        self._last_cmd: Optional[VectorSample] = None
        self._last_js: Optional[VectorSample] = None

        self._warned_no_state = False
        self._tick_count = 0

        self.create_subscription(
            JointTrajectoryControllerState, self.controller_state_topic, self._on_state, 10
        )
        self.create_subscription(JointTrajectory, self.command_topic, self._on_command, 10)
        self.create_subscription(JointState, self.joint_states_topic, self._on_joint_states, 10)

        period = 1.0 / max(0.5, self.print_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info("JointTrackingMonitor started")
        self.get_logger().info(f"  controller_state_topic: {self.controller_state_topic}")
        self.get_logger().info(f"  command_topic:          {self.command_topic}")
        self.get_logger().info(f"  joint_states_topic:     {self.joint_states_topic}")
        self.get_logger().info(f"  joints:                 {self.arm_joint_names}")

    def _on_state(self, msg: JointTrajectoryControllerState):
        # The message should already be aligned by joint order in msg.joint_names.
        names = list(msg.joint_names)
        idx = _index_map(names, self.arm_joint_names)
        if idx is None:
            self.get_logger().warn(
                f"Controller state missing one of arm joints. Have={names}, expected={self.arm_joint_names}"
            )
            return

        t = _now_s(self)
        desired = [float(msg.desired.positions[i]) for i in idx]
        actual = [float(msg.actual.positions[i]) for i in idx]
        error = [d - a for d, a in zip(desired, actual)]
        actual_vel = None
        try:
            if msg.actual.velocities and len(msg.actual.velocities) >= len(names):
                actual_vel = [float(msg.actual.velocities[i]) for i in idx]
        except Exception:
            actual_vel = None

        self._last_desired = VectorSample(t, self.arm_joint_names, desired)
        self._last_actual = VectorSample(t, self.arm_joint_names, actual)
        self._last_error = VectorSample(t, self.arm_joint_names, error)
        if actual_vel is not None:
            self._last_actual_vel = VectorSample(t, self.arm_joint_names, actual_vel)

    def _on_command(self, msg: JointTrajectory):
        if not msg.points:
            return
        # Use the last point (most relevant target)
        p = msg.points[-1]
        names = list(msg.joint_names)
        idx = _index_map(names, self.arm_joint_names)
        if idx is None:
            return
        if len(p.positions) < len(names):
            return
        t = _now_s(self)
        cmd = [float(p.positions[i]) for i in idx]
        self._last_cmd = VectorSample(t, self.arm_joint_names, cmd)

    def _on_joint_states(self, msg: JointState):
        names = list(msg.name)
        idx = _index_map(names, self.arm_joint_names)
        if idx is None:
            return
        if len(msg.position) < len(names):
            return
        t = _now_s(self)
        q = [float(msg.position[i]) for i in idx]
        self._last_js = VectorSample(t, self.arm_joint_names, q)

    def _rms(self, v: List[float]) -> float:
        if not v:
            return 0.0
        return math.sqrt(sum(x * x for x in v) / float(len(v)))

    def _tick(self):
        self._tick_count += 1
        # Preferred path: controller state gives desired/actual directly.
        if self._last_error is not None and self._last_desired is not None and self._last_actual is not None:
            err = self._last_error.values
            max_abs = max(abs(x) for x in err) if err else 0.0
            rms = self._rms(err)

            status = "OK"
            if max_abs > self.max_abs_pos_error_rad_warn:
                status = "WARN"

            # Stall heuristic: large error AND not moving (requires actual velocities)
            stall = False
            per_joint_stall = {}
            if self._last_actual_vel is not None and len(self._last_actual_vel.values) == len(err):
                for name, e, v in zip(self.arm_joint_names, err, self._last_actual_vel.values):
                    if abs(e) > self.stall_error_rad and abs(v) < self.stall_vel_eps:
                        per_joint_stall[name] = (e, v)
                stall = len(per_joint_stall) > 0

            line = (
                f"[{status}] err_rms={rms:.3f} rad, err_max={max_abs:.3f} rad "
                f"(desired vs actual)"
            )
            if stall and status == "WARN":
                stalled = ", ".join([f"{n}(e={e:+.3f},v={v:+.3f})" for n, (e, v) in per_joint_stall.items()])
                line += f" possible STALL [{stalled}]"

            # Compact per-joint summary when warning
            if status == "WARN":
                # Include desired/actual snapshots for the biggest-error joints (top 2)
                pairs: List[Tuple[str, float]] = list(zip(self.arm_joint_names, err))
                pairs.sort(key=lambda x: abs(x[1]), reverse=True)
                top = [n for n, _ in pairs[:2]]
                desired = self._last_desired.values
                actual = self._last_actual.values
                da = ", ".join(
                    f"{n}(des={desired[self.arm_joint_names.index(n)]:+.3f},act={actual[self.arm_joint_names.index(n)]:+.3f})"
                    for n in top
                )
                per = ", ".join(
                    f"{n}={e:+.3f}" for n, e in zip(self.arm_joint_names, err)
                )
                line += f" | err: {per} | top: {da}"
                # Throttle WARN spam a bit (still prints at most ~2.5Hz with default 5Hz)
                if self._tick_count % 2 == 0:
                    self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
            return

        # Fallback path: compare last command vs joint_states
        if self._last_cmd is None or self._last_js is None:
            if not self._warned_no_state:
                self.get_logger().warn(
                    "No controller state yet. Waiting for /state; "
                    "fallback requires both command and joint_states."
                )
                self._warned_no_state = True
            return

        cmd = self._last_cmd.values
        q = self._last_js.values
        err = [c - a for c, a in zip(cmd, q)]
        max_abs = max(abs(x) for x in err) if err else 0.0
        rms = self._rms(err)

        status = "OK" if max_abs <= self.max_abs_pos_error_rad_warn else "WARN"
        line = f"[{status}] err_rms={rms:.3f} rad, err_max={max_abs:.3f} rad (cmd vs joint_states)"
        if status == "WARN":
            per = ", ".join(f"{n}={e:+.3f}" for n, e in zip(self.arm_joint_names, err))
            self.get_logger().warn(line + f" | err: {per}")
        else:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrackingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # Can happen if shutdown already occurred via signal handling
            pass


