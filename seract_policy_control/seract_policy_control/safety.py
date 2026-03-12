import numpy as np
import time
import logging

# Set up logger for safety module
logger = logging.getLogger(__name__)


class Safety:
    def __init__(self, qmin, qmax, max_step=0.10, js_timeout=1.5):
        self.qmin = np.array(qmin)
        self.qmax = np.array(qmax)
        self.max_step = float(max_step)  # Maximum step size in radians
        self.js_timeout = float(js_timeout)
        self.last_js_t = 0.0
        self.estop = False
        
        # Log initialization
        logger.info(f"Safety initialized: max_step={self.max_step} rad, qmin={self.qmin}, qmax={self.qmax}")

    def mark_js(self):
        self.last_js_t = time.time()

    def ok(self):
        return (not self.estop) and (time.time() - self.last_js_t < self.js_timeout)

    def step_limit(self, q_curr, q_cmd):
        """
        Apply step limiting to prevent large joint movements.
        
        Args:
            q_curr: Current joint positions (radians)
            q_cmd: Commanded joint positions (radians)
        
        Returns:
            Joint positions after step limiting (radians)
        """
        q_curr = np.asarray(q_curr)
        q_cmd = np.asarray(q_cmd)
        dq_raw = q_cmd - q_curr
        dq_clamped = np.clip(dq_raw, -self.max_step, self.max_step)
        q_cmd_final = q_curr + dq_clamped
        
        # Log step limiting
        logger.debug(f"STEP_LIMIT: q_curr={q_curr}, q_cmd_raw={q_cmd}, dq_clamped={dq_clamped}, q_cmd_final={q_cmd_final}")
        
        if np.any(np.abs(dq_raw) > self.max_step):
            limited_joints = np.where(np.abs(dq_raw) > self.max_step)[0]
            logger.debug(f"STEP_LIMIT: Applied to joints {limited_joints} (max_step={self.max_step} rad)")
        
        return q_cmd_final

    def joint_limits(self, q):
        """
        Apply joint limits to keep joints within safe range.
        
        Args:
            q: Joint positions (radians)
        
        Returns:
            Joint positions clipped to [qmin, qmax] (radians)
        """
        q = np.asarray(q)
        q_before = q.copy()
        q_after = np.clip(q, self.qmin, self.qmax)
        
        # Log joint limiting
        logger.debug(f"JOINT_LIMITS: q_before={q_before}, q_after={q_after}")
        
        if np.any((q_before < self.qmin) | (q_before > self.qmax)):
            limited_joints = np.where((q_before < self.qmin) | (q_before > self.qmax))[0]
            logger.debug(f"JOINT_LIMITS: Applied to joints {limited_joints}")
        
        return q_after
    
    def process_command(self, q_curr, q_cmd_raw):
        """
        Process a joint command with both step limiting and joint limits.
        Applies safety checks in order: step_limit -> joint_limits
        
        Args:
            q_curr: Current joint positions (radians) [N]
            q_cmd_raw: Raw commanded joint positions (radians) [N]
        
        Returns:
            q_cmd_final: Final safe joint command (radians) [N]
        """
        # Step 1: Apply step limiting
        dq_raw = q_cmd_raw - q_curr
        dq_clamped = np.clip(dq_raw, -self.max_step, self.max_step)
        q_cmd_after_step = q_curr + dq_clamped
        
        # Step 2: Apply joint limits
        q_cmd_final = np.clip(q_cmd_after_step, self.qmin, self.qmax)
        
        # Detailed logging (debug only - too verbose for normal operation)
        logger.debug("=" * 60)
        logger.debug("SAFETY: Processing new command")
        logger.debug(f"  q_curr (rad):      {q_curr}")
        logger.debug(f"  q_cmd_raw (rad):   {q_cmd_raw}")
        logger.debug(f"  dq_raw (rad):      {dq_raw}")
        logger.debug(f"  dq_clamped (rad):  {dq_clamped} (max_step={self.max_step} rad)")
        logger.debug(f"  q_cmd_after_step:  {q_cmd_after_step}")
        logger.debug(f"  q_cmd_final (rad): {q_cmd_final}")
        
        # Check if any limits were applied
        step_limited = np.any(np.abs(dq_raw) > self.max_step)
        joint_limited = np.any((q_cmd_after_step < self.qmin) | (q_cmd_after_step > self.qmax))
        
        if step_limited:
            limited_joints = np.where(np.abs(dq_raw) > self.max_step)[0]
            logger.debug(f"  Step limit applied to joints: {limited_joints}")
        
        if joint_limited:
            limited_joints = np.where((q_cmd_after_step < self.qmin) | (q_cmd_after_step > self.qmax))[0]
            logger.debug(f"  Joint limit applied to joints: {limited_joints}")
        
        logger.debug("=" * 60)
        
        return q_cmd_final
