import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class JointJogger(Node):
    def __init__(self):
        super().__init__('joint_jogger')
        # explicit param types
        str_arr = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        dbl_arr = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)

        self.declare_parameter('controller_topic', '/so_100_arm_controller/joint_trajectory')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('command_dt', 0.25)   # seconds horizon for each point
        self.declare_parameter('joint_names', [], descriptor=str_arr)
        self.declare_parameter('joint_min', [], descriptor=dbl_arr)
        self.declare_parameter('joint_max', [], descriptor=dbl_arr)
        self.declare_parameter('jog_joint_index', 0) # which joint to jog (single joint mode)
        self.declare_parameter('jog_joint_indices', []) # which joints to jog (multi-joint mode)
        self.declare_parameter('jog_amplitude', 0.05) # radians peak
        self.declare_parameter('jog_frequency', 0.2) # Hz
        self.declare_parameter('max_step_rad', 0.05) # safety step clamp

        gp = lambda k: self.get_parameter(k).get_parameter_value()
        self.controller_topic = gp('controller_topic').string_value
        self.rate_hz = gp('rate_hz').double_value
        self.command_dt = gp('command_dt').double_value
        self.joint_names = list(gp('joint_names').string_array_value)
        jm = gp('joint_min').double_array_value
        jM = gp('joint_max').double_array_value
        self.joint_min = np.array(jm, dtype=np.float64) if len(jm) else None
        self.joint_max = np.array(jM, dtype=np.float64) if len(jM) else None
        self.jog_idx = gp('jog_joint_index').integer_value
        # Try to read jog_joint_indices parameter
        try:
            jog_indices_param = self.get_parameter('jog_joint_indices')
            if jog_indices_param.type_ == ParameterType.PARAMETER_INTEGER_ARRAY:
                self.jog_indices = list(jog_indices_param.get_parameter_value().integer_array_value)
            else:
                self.jog_indices = []
        except Exception:
            self.jog_indices = []
        self.jog_amp = gp('jog_amplitude').double_value
        self.jog_freq = gp('jog_frequency').double_value
        self.max_step = gp('max_step_rad').double_value

        self.last_joints = None
        self.phase0 = time.time()
        self.q_center = None  # Stored center/reference position for oscillation (in radians)
        self._log_count = 0  # Counter for logging frequency

        qos = QoSProfile(depth=10)
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.on_js, qos)
        self.pub_traj = self.create_publisher(JointTrajectory, self.controller_topic, qos)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(f"JointJogger → publishing to {self.controller_topic} at {self.rate_hz} Hz")

    def on_js(self, msg: JointState):
        # Controller expects joints in this exact order (from controllers.yaml):
        # Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll
        # We enforce this order explicitly to match the controller, regardless of /joint_states order
        EXPECTED_JOINT_ORDER = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
        
        # Auto-discover joint names and ordering on first message if needed
        if not self.joint_names:
            # Use the controller's expected order explicitly
            # This ensures joint_names and positions always match controller expectations
            all_joints = list(msg.name)
            available_joints = set(all_joints)
            
            # Verify all expected joints are present
            missing = [j for j in EXPECTED_JOINT_ORDER if j not in available_joints]
            if missing:
                self.get_logger().error(f"Missing required joints in /joint_states: {missing}")
                self.get_logger().error(f"Available joints: {all_joints}")
                return
            
            # Use controller's expected order (not /joint_states order)
            self.joint_names = EXPECTED_JOINT_ORDER.copy()
            n = len(self.joint_names)
            if self.joint_min is None or self.joint_min.size == 0:
                self.joint_min = np.full(n, -np.pi, dtype=np.float64)
            if self.joint_max is None or self.joint_max.size == 0:
                self.joint_max = np.full(n,  np.pi, dtype=np.float64)
            self.get_logger().info(f"✓ Joint names set to controller order: {self.joint_names}")
            self.get_logger().info(f"  /joint_states order: {all_joints}")
        
        # Map positions from /joint_states (order may differ) to controller order
        name_to_pos = {n:p for n,p in zip(msg.name, msg.position)}
        try:
            # Extract positions in controller's expected order (self.joint_names)
            self.last_joints = np.array([name_to_pos[n] for n in self.joint_names], dtype=np.float64)
        except KeyError as e:
            missing = [n for n in self.joint_names if n not in name_to_pos]
            self.get_logger().warn(f"Missing joints in /joint_states: {missing}")

    def clamp(self, q):
        if self.joint_min is not None and self.joint_max is not None and \
           self.joint_min.size == q.size and self.joint_max.size == q.size:
            return np.clip(q, self.joint_min, self.joint_max)
        return q

    def tick(self):
        if self.last_joints is None:
            self.get_logger().warn("Waiting for /joint_states...")
            return
        # Determine which joints to jog
        # For now, hardcode 3 joints: 0, 1, 2 (Shoulder_Rotation, Shoulder_Pitch, Elbow)
        if len(self.jog_indices) > 0:
            joints_to_jog = self.jog_indices
        elif self.jog_idx == 999:  # Special value to jog 3 joints
            joints_to_jog = [0, 1, 2]
        else:
            joints_to_jog = [self.jog_idx]
        
        # Validate joint indices
        valid_joints = [idx for idx in joints_to_jog if 0 <= idx < len(self.joint_names)]
        if not valid_joints:
            self.get_logger().warn(f"No valid joints to jog; doing nothing")
            return

        # Calculate elapsed time since start (in seconds)
        t = time.time() - self.phase0
        
        # Store center/reference position on first tick (in radians)
        # This is the position around which we oscillate
        if self.q_center is None:
            self.q_center = self.last_joints.copy()
            self.get_logger().info(f"📌 [JOGGER] Stored center positions (rad): {self.q_center}")
        
        q_des = self.last_joints.copy()
        
        # Jog each joint with a phase offset for smoother motion
        for i, joint_idx in enumerate(joints_to_jog):
            if joint_idx < 0 or joint_idx >= len(self.joint_names):
                continue
            
            # Phase offset: spread joints evenly across the cycle (in radians)
            phase_offset = (2.0 * math.pi * i) / len(joints_to_jog) if len(joints_to_jog) > 1 else 0.0
            
            # Calculate target position offset using sinusoidal motion (all values in radians):
            #   target_offset = amplitude * sin(2π * frequency * time + phase_offset)
            #   where:
            #     - amplitude (jog_amp) is in radians
            #     - frequency (jog_freq) is in Hz (cycles per second)
            #     - time (t) is in seconds
            #     - phase_offset is in radians
            #   Result: target_offset oscillates between -amplitude and +amplitude (in radians)
            target_offset = self.jog_amp * math.sin(2.0 * math.pi * self.jog_freq * t + phase_offset)
            
            # Calculate target position by adding sine offset to stored center position (in radians)
            #   target_pos = q_center + target_offset
            target_pos = self.q_center[joint_idx] + target_offset
            
            # Calculate delta from CURRENT position to target (in radians)
            delta = target_pos - self.last_joints[joint_idx]
            
            # Clamp delta step size for safety (but allow gradual approach to target)
            delta = max(-self.max_step, min(self.max_step, delta))
            
            # Apply delta to desired position
            q_des[joint_idx] = self.last_joints[joint_idx] + delta
        
        # Log current and target positions periodically (every 10 cycles at 10 Hz = ~1 second)
        self._log_count += 1
        if self._log_count % 10 == 0:
            jogging_info = []
            for i, joint_idx in enumerate(joints_to_jog):
                if 0 <= joint_idx < len(self.joint_names):
                    current = self.last_joints[joint_idx]
                    target = q_des[joint_idx]
                    center = self.q_center[joint_idx]
                    # Recalculate ideal offset for logging
                    phase_offset = (2.0 * math.pi * i) / len(joints_to_jog) if len(joints_to_jog) > 1 else 0.0
                    ideal_offset = self.jog_amp * math.sin(2.0 * math.pi * self.jog_freq * t + phase_offset)
                    actual_offset = target - center
                    delta = target - current
                    jogging_info.append(
                        f"{self.joint_names[joint_idx]}: "
                        f"curr={current:.4f} rad, "
                        f"target={target:.4f} rad, "
                        f"center={center:.4f} rad, "
                        f"ideal_offset={ideal_offset:.4f} rad, "
                        f"actual_offset={actual_offset:.4f} rad, "
                        f"delta={delta:.4f} rad"
                    )
            self.get_logger().info(f"🔄 [JOGGER] Cycle {self._log_count} (t={t:.2f}s): {', '.join(jogging_info)}")

        q_des = self.clamp(q_des)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        # Set header timestamp to current time (controller expects this)
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = ''
        
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_des.tolist()]
        # Controller requires last point to have zero velocity (safety requirement)
        # Since we're publishing single-point trajectories, always set velocities to zero
        pt.velocities = [0.0] * len(q_des)
        
        # time_from_start: time from trajectory start to reach this point
        # For single-point trajectories, this should be > 0 (controller requirement)
        pt.time_from_start.sec = int(self.command_dt)
        pt.time_from_start.nanosec = int((self.command_dt - int(self.command_dt)) * 1e9)
        traj.points = [pt]

        self.pub_traj.publish(traj)

def main():
    rclpy.init()
    node = JointJogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

