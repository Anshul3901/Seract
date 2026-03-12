#!/usr/bin/env python3
import os
import json
import time
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from lifecycle_msgs.msg import State
from std_srvs.srv import SetBool
from controller_manager_msgs.srv import (
    SwitchController,
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
)
from so101_motion_interfaces.action import DecisionAct


class DecisionMovementActionServer(Node):
    # -----------------------------
    # CONFIG — EDIT THESE IF NEEDED
    # -----------------------------
    DECISION_PATH = "/home/gixadmin/seract_orch_ws/src/so101_motion/so101_motion/decision.json"
    POLL_PERIOD_SEC = 0.5

    # ROS2 controller handoff (no pkill): deactivate for ACT, reactivate after ACT
    ARM_CONTROLLERS = ["so_100_arm_controller", "so_100_arm_gripper_controller"]
    SWITCH_TIMEOUT_SEC = 8.0
    STATE_WAIT_TIMEOUT_SEC = 8.0
    SERVICE_WAIT_TIMEOUT_SEC = 10.0
    POST_SCRIPT_WAIT_SEC = 12.0
    HARDWARE_COMPONENT_NAME = "SO100ARM"
    HARDWARE_RELEASE_STATE_ID = State.PRIMARY_STATE_UNCONFIGURED
    HARDWARE_RELEASE_STATE_LABEL = "unconfigured"
    HARDWARE_RELEASE_SETTLE_SEC = 1.5
    USB_CAMERA_ENABLE_SERVICE = "/usb_camera_node/set_enabled"

    # Policy scripts
    POLICY_DIR = "/home/gixadmin/ai_policy"
    POLICY_ACCEPT = ["bash", "./run_accept_march5_direct.sh"]
    POLICY_REJECT = ["bash", "./run_reject_feb_direct.sh"]

    # Logging
    LOG_DIR = "/home/gixadmin/seract_orch_ws/policy_logs"

    def __init__(self):
        super().__init__("decision_movement_action_server")

        os.makedirs(self.LOG_DIR, exist_ok=True)

        self._cb_group = ReentrantCallbackGroup()

        # Use absolute name to avoid namespace surprises
        self._action_name = "/decision_movement/run"

        self._action_server = ActionServer(
            self,
            DecisionAct,
            self._action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self._switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._list_hw_client = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self._set_hw_state_client = self.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )
        self._usb_camera_enable_client = self.create_client(
            SetBool, self.USB_CAMERA_ENABLE_SERVICE
        )

        self.get_logger().info(f"✅ DecisionMovement Action Server ready: {self._action_name}")
        self.get_logger().info(f"📄 Decision file: {self.DECISION_PATH}")
        self.get_logger().info(
            f"🎛️ Controller handoff enabled for: {', '.join(self.ARM_CONTROLLERS)}"
        )
        self.get_logger().info(
            f"🧩 Hardware release target for policy handoff: {self.HARDWARE_COMPONENT_NAME}"
        )
        self.get_logger().info(
            f"📷 USB camera control service: {self.USB_CAMERA_ENABLE_SERVICE}"
        )

    # -----------------------------
    # Action callbacks
    # -----------------------------
    def goal_callback(self, goal_request: DecisionAct.Goal) -> GoalResponse:
        self.get_logger().info(f"🎯 Goal received: wait_for_decision={goal_request.wait_for_decision}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("🛑 Cancel requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = DecisionAct.Feedback()
        result = DecisionAct.Result()

        def publish(state: str, progress: float):
            feedback.state = state
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

        publish("STARTING", 0.0)

        # 1) Determine decision
        decision_norm = ""
        publish("CHECKING_DECISION", 0.05)

        if goal_handle.request.wait_for_decision:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.decision = ""
                    result.message = "Canceled while waiting for decision"
                    return result

                d = self._read_decision()
                if d is not None:
                    decision_norm = self._normalize_decision(d)
                    if decision_norm in ("accept", "reject"):
                        break
                time.sleep(self.POLL_PERIOD_SEC)
        else:
            d = self._read_decision()
            decision_norm = self._normalize_decision(d or "")

        if decision_norm not in ("accept", "reject"):
            goal_handle.abort()
            result.success = False
            result.decision = ""
            result.message = "No valid decision found (need accept/reject)"
            return result

        self.get_logger().warn(f"🔥 Decision: {decision_norm}")
        result.decision = decision_norm

        # Clear decision to avoid re-trigger elsewhere
        self._clear_decision()

        # 2) ROS2-native handoff: deactivate controllers -> run policy script -> reactivate controllers
        controllers_deactivated = False
        usb_camera_disabled = False
        try:
            publish("DEACTIVATING_CONTROLLERS", 0.2)
            ok, msg = self._handoff_to_act()
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = f"Controller handoff failed: {msg}"
                return result
            controllers_deactivated = True

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Canceled after controller handoff"
                return result

            # Release /dev/video6 for ACT policy camera use.
            ok_cam, msg_cam = self._set_usb_camera_enabled(False, timeout_sec=5.0)
            if not ok_cam:
                self.get_logger().warn(f"⚠️ Could not disable USB camera before policy: {msg_cam}")
            else:
                usb_camera_disabled = True

            publish("RUNNING_SCRIPT", 0.5)
            script_cmd, script_name = self._policy_command_for_decision(decision_norm)
            self.get_logger().info(
                f"🎬 RUNNING_SCRIPT -> decision='{decision_norm}' command='{' '.join(script_cmd)}' cwd='{self.POLICY_DIR}'"
            )
            rc = self._run_script_blocking(
                script_cmd,
                cwd=self.POLICY_DIR,
                name=script_name,
                goal_handle=goal_handle,
            )

            if goal_handle.is_cancel_requested or rc == 130:
                goal_handle.canceled()
                result.success = False
                result.message = "Canceled while running script"
                return result

            if rc != 0:
                goal_handle.abort()
                result.success = False
                result.message = f"Script failed (exit code {rc})"
                return result

            publish("POST_SCRIPT_WAIT", 0.7)
            self.get_logger().info(
                f"⏳ Waiting {self.POST_SCRIPT_WAIT_SEC:.1f}s after script before reactivating controllers..."
            )
            if not self._wait_with_cancel(goal_handle, self.POST_SCRIPT_WAIT_SEC):
                goal_handle.canceled()
                result.success = False
                result.message = "Canceled during post-script wait"
                return result

            publish("REACTIVATING_CONTROLLERS", 0.85)
            ok, msg = self._restore_ros_control()
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = f"Failed to restore ROS controllers: {msg}"
                return result
            controllers_deactivated = False

            if usb_camera_disabled:
                ok_cam, msg_cam = self._set_usb_camera_enabled(True, timeout_sec=8.0)
                if not ok_cam:
                    self.get_logger().error(f"❌ Failed to re-enable USB camera after policy: {msg_cam}")
                else:
                    usb_camera_disabled = False

            publish("DONE", 1.0)
            goal_handle.succeed()
            result.success = True
            result.message = "Done"
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"Failed: {e}"
            return result
        finally:
            # Best-effort safety: if something failed after handoff, try restoring ROS controller ownership.
            if controllers_deactivated:
                ok, msg = self._restore_ros_control()
                if not ok:
                    self.get_logger().error(f"❌ Emergency restore failed: {msg}")
            if usb_camera_disabled:
                ok_cam, msg_cam = self._set_usb_camera_enabled(True, timeout_sec=8.0)
                if not ok_cam:
                    self.get_logger().error(f"❌ Emergency USB camera re-enable failed: {msg_cam}")

    def _set_usb_camera_enabled(self, enabled: bool, timeout_sec: float = 5.0):
        if not self._wait_for_service(
            self._usb_camera_enable_client,
            self.USB_CAMERA_ENABLE_SERVICE,
            timeout_sec,
        ):
            return False, "usb camera enable service unavailable"

        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._usb_camera_enable_client.call_async(req)
        resp = self._wait_for_future(fut, timeout_sec)
        if resp is None:
            return False, "usb camera enable call timed out"
        if not bool(getattr(resp, "success", False)):
            return False, str(getattr(resp, "message", "unknown error"))
        return True, str(getattr(resp, "message", "ok"))

    # -----------------------------
    # Decision helpers
    # -----------------------------
    def _read_decision(self) -> Optional[str]:
        if not os.path.exists(self.DECISION_PATH):
            return None
        try:
            with open(self.DECISION_PATH, "r") as f:
                data = json.load(f)
            d = str(data.get("decision", "")).strip()
            return d if d else None
        except Exception:
            return None

    def _clear_decision(self):
        try:
            with open(self.DECISION_PATH, "w") as f:
                json.dump({"decision": ""}, f)
        except Exception:
            pass

    def _normalize_decision(self, d: str) -> str:
        dl = (d or "").lower().strip()
        if dl in ["restock / resell", "restock/resell", "restock", "resell", "accept", "approved"]:
            return "accept"
        if dl in ["recycle / review", "recycle/review", "recycle", "review", "reject", "denied"]:
            return "reject"
        return ""

    def _policy_command_for_decision(self, decision_norm: str):
        if decision_norm == "accept":
            cmd = self.POLICY_ACCEPT.copy()
            if len(cmd) >= 2 and not os.path.isabs(cmd[1]):
                cmd[1] = os.path.join(self.POLICY_DIR, cmd[1])
            return cmd, "policy_accept"
        if decision_norm == "reject":
            cmd = self.POLICY_REJECT.copy()
            if len(cmd) >= 2 and not os.path.isabs(cmd[1]):
                cmd[1] = os.path.join(self.POLICY_DIR, cmd[1])
            return cmd, "policy_reject"
        raise ValueError(f"Unsupported decision for policy run: '{decision_norm}'")

    # -----------------------------
    # ROS2 control helpers
    # -----------------------------
    def _wait_for_future(self, future, timeout_sec: float):
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def _wait_with_cancel(self, goal_handle, timeout_sec: float) -> bool:
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False
            time.sleep(0.2)
        return True

    def _wait_for_service(self, client, service_name: str, timeout_sec: float) -> bool:
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            if client.wait_for_service(timeout_sec=0.3):
                return True
        self.get_logger().error(f"Service not available: {service_name}")
        return False

    def _switch_controllers(self, activate, deactivate, strict: bool = True):
        if not self._wait_for_service(
            self._switch_client, "/controller_manager/switch_controller", self.SERVICE_WAIT_TIMEOUT_SEC
        ):
            return False, "switch_controller service unavailable"

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = 2 if strict else 1  # STRICT=2, BEST_EFFORT=1

        fut = self._switch_client.call_async(req)
        resp = self._wait_for_future(fut, self.SWITCH_TIMEOUT_SEC)
        if resp is None:
            return False, "switch_controller timeout"

        if not getattr(resp, "ok", False):
            return False, "switch_controller responded with ok=False"

        return True, "ok"

    def _list_controller_states(self):
        if not self._wait_for_service(
            self._list_client, "/controller_manager/list_controllers", self.SERVICE_WAIT_TIMEOUT_SEC
        ):
            return None

        req = ListControllers.Request()
        fut = self._list_client.call_async(req)
        resp = self._wait_for_future(fut, self.SWITCH_TIMEOUT_SEC)
        if resp is None:
            return None
        return {c.name: c.state for c in resp.controller}

    def _list_hardware_states(self):
        if not self._wait_for_service(
            self._list_hw_client,
            "/controller_manager/list_hardware_components",
            self.SERVICE_WAIT_TIMEOUT_SEC,
        ):
            return None

        req = ListHardwareComponents.Request()
        fut = self._list_hw_client.call_async(req)
        resp = self._wait_for_future(fut, self.SWITCH_TIMEOUT_SEC)
        if resp is None:
            return None
        return {c.name: c.state for c in resp.component}

    def _resolve_hardware_component_name(self) -> Optional[str]:
        states = self._list_hardware_states()
        if not states:
            return None

        # Prefer configured explicit name.
        if self.HARDWARE_COMPONENT_NAME in states:
            return self.HARDWARE_COMPONENT_NAME

        # Case-insensitive fallback.
        cfg_lower = self.HARDWARE_COMPONENT_NAME.lower()
        for name in states.keys():
            if name.lower() == cfg_lower:
                return name

        # Final fallback: single hardware component system.
        if len(states) == 1:
            return next(iter(states.keys()))

        self.get_logger().error(
            "Unable to resolve hardware component name. Available: "
            + ", ".join(states.keys())
        )
        return None

    def _wait_hardware_state(self, hw_name: str, target_id: int, timeout_sec: float) -> bool:
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            states = self._list_hardware_states()
            if states is not None:
                s = states.get(hw_name, None)
                if s is not None and int(getattr(s, "id", -1)) == int(target_id):
                    return True
            time.sleep(0.2)
        return False

    def _set_hardware_state(self, hw_name: str, target_id: int, label: str):
        if not self._wait_for_service(
            self._set_hw_state_client,
            "/controller_manager/set_hardware_component_state",
            self.SERVICE_WAIT_TIMEOUT_SEC,
        ):
            return False, "set_hardware_component_state service unavailable"

        req = SetHardwareComponentState.Request()
        req.name = hw_name
        req.target_state = State(id=int(target_id), label=label)

        fut = self._set_hw_state_client.call_async(req)
        resp = self._wait_for_future(fut, self.SWITCH_TIMEOUT_SEC)
        if resp is None:
            return False, "set_hardware_component_state timeout"
        if not getattr(resp, "ok", False):
            current = getattr(resp, "state", None)
            current_id = getattr(current, "id", -1) if current is not None else -1
            current_label = getattr(current, "label", "") if current is not None else ""
            return False, f"set_hardware_component_state refused (current={current_id}:{current_label})"
        return True, "ok"

    def _wait_controllers_state(self, names, target_state: str, timeout_sec: float) -> bool:
        deadline = time.time() + float(timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            states = self._list_controller_states()
            if states is not None:
                if all(states.get(n, "") == target_state for n in names):
                    return True
            time.sleep(0.2)
        return False

    def _handoff_to_act(self):
        self.get_logger().warn("🔌 Deactivating ROS arm controllers for ACT handoff...")
        ok, msg = self._switch_controllers(activate=[], deactivate=self.ARM_CONTROLLERS, strict=True)
        if not ok:
            return False, msg
        if not self._wait_controllers_state(
            self.ARM_CONTROLLERS, "inactive", self.STATE_WAIT_TIMEOUT_SEC
        ):
            return False, "controllers did not reach inactive state in time"

        hw_name = self._resolve_hardware_component_name()
        if not hw_name:
            return False, "could not resolve hardware component name"

        release_id = int(self.HARDWARE_RELEASE_STATE_ID)
        release_label = str(self.HARDWARE_RELEASE_STATE_LABEL)

        self.get_logger().warn(
            f"🧩 Releasing hardware component '{hw_name}' to state {release_label} for policy serial access..."
        )
        ok, msg = self._set_hardware_state(
            hw_name,
            release_id,
            release_label,
        )
        if not ok:
            return False, msg
        if not self._wait_hardware_state(hw_name, release_id, self.STATE_WAIT_TIMEOUT_SEC):
            return False, f"hardware did not reach {release_label} state in time"

        # Allow serial line and motor bus to settle before policy handshake.
        time.sleep(float(self.HARDWARE_RELEASE_SETTLE_SEC))

        self.get_logger().info("✅ Controllers inactive and hardware released. ACT can take control.")
        return True, "ok"

    def _restore_ros_control(self):
        hw_name = self._resolve_hardware_component_name()
        if not hw_name:
            return False, "could not resolve hardware component name"

        self.get_logger().warn(f"🔁 Re-acquiring hardware component '{hw_name}' after ACT...")
        ok, msg = self._set_hardware_state(
            hw_name,
            State.PRIMARY_STATE_ACTIVE,
            "active",
        )
        if not ok:
            return False, msg
        if not self._wait_hardware_state(hw_name, State.PRIMARY_STATE_ACTIVE, self.STATE_WAIT_TIMEOUT_SEC):
            return False, "hardware did not reach active state in time"

        self.get_logger().warn("🔁 Reactivating ROS arm controllers after ACT...")
        ok, msg = self._switch_controllers(activate=self.ARM_CONTROLLERS, deactivate=[], strict=True)
        if not ok:
            return False, msg
        if not self._wait_controllers_state(
            self.ARM_CONTROLLERS, "active", self.STATE_WAIT_TIMEOUT_SEC
        ):
            return False, "controllers did not reach active state in time"
        self.get_logger().info("✅ ROS arm controllers restored and active.")
        return True, "ok"

    def _run_script_blocking(self, cmd, cwd=None, name="script", goal_handle=None) -> int:
        ts = int(time.time())
        out_path = os.path.join(self.LOG_DIR, f"{name}_stdout_{ts}.log")
        err_path = os.path.join(self.LOG_DIR, f"{name}_stderr_{ts}.log")

        self.get_logger().info(f"🚀 Running {name}: {' '.join(cmd)}")
        self.get_logger().info(f"📝 Logs:\n  stdout: {out_path}\n  stderr: {err_path}")

        with open(out_path, "w") as out_f, open(err_path, "w") as err_f:
            # Not creating a new process group here because we want a simple "run to completion"
            proc = subprocess.Popen(
                cmd,
                cwd=cwd,
                stdout=out_f,
                stderr=err_f,
                text=True,
                env=os.environ.copy(),
            )

            # Wait while staying cancel-responsive
            while rclpy.ok():
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    try:
                        proc.terminate()
                        proc.wait(timeout=3.0)
                    except Exception:
                        try:
                            proc.kill()
                        except Exception:
                            pass
                    return 130

                rc = proc.poll()
                if rc is not None:
                    return int(rc)
                time.sleep(0.2)

        return 1  # should not reach

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMovementActionServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
