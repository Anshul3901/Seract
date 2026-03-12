import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2
import os
import time


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        
        # Policy bridge enable publisher
        self._policy_enable_pub = self.create_publisher(
            Bool,
            '/policy_bridge/enable',
            10
        )

        # Arm action client
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/so_100_arm_controller/follow_joint_trajectory'
        )

        # Gripper publisher
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/so_100_arm_gripper_controller/commands',
            10
        )

        # Image subscriber
        self._image_sub = self.create_subscription(
            Image,
            '/camera2/color/image_raw',
            self._image_cb,
            10
        )

        self.bridge = CvBridge()
        self.latest_image = None

        # Save directory
        self.save_dir = os.path.expanduser('~/seract_orch_ws/scanned_pictures')
        os.makedirs(self.save_dir, exist_ok=True)

        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]

        self.get_logger().info("✅ PickPlaceNode ready.")

    # -------------------------------
    # Image callback
    # -------------------------------
    def _image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    # -------------------------------
    # Capture one image
    # -------------------------------
    def capture_image(self, tag):
        self.get_logger().info(f"📸 Capturing image: {tag}")

        # Wait for a fresh image
        timeout = time.time() + 3.0
        while self.latest_image is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_image is None:
            self.get_logger().error("❌ No image received.")
            return

        filename = os.path.join(self.save_dir, f"{tag}.png")
        cv2.imwrite(filename, self.latest_image)
        self.get_logger().info(f"✅ Saved image to {filename}")

    # -------------------------------
    # Helper: send trajectory
    # -------------------------------
    def move_arm(self, positions, duration=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending trajectory: {positions}")
        self._arm_client.wait_for_server()
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Arm goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ Arm motion complete.')

    # -------------------------------
    # Helper: control gripper
    # -------------------------------
    def control_gripper(self, value):
        msg = Float64MultiArray()
        msg.data = [value]
        self._gripper_pub.publish(msg)
        action = "Closing" if value < 0 else "Opening"
        self.get_logger().info(f"{action} gripper with command: {value}")
        time.sleep(1.0)

    # -------------------------------
    # Full task sequence
    # -------------------------------
    def execute_task(self):
        self.get_logger().info("🚀 Starting pick and rotate sequence.")
        
        time.sleep(20.0)
        self.control_gripper(1.0)
        time.sleep(12.0)

        # Top
        self.move_arm([
            0.60632831592921,
            -0.5579790940766551,
            -2.325107810253953,
            0.5193631669535284,
            -2.2986225895316803
        ])
        time.sleep(3.0)
        self.capture_image("top")
        time.sleep(12.0)

        # Right
        self.move_arm([
            1.0802103772240566,
            0.6517488021902806,
            -0.8624820316243411,
            -1.6759896729776247,
            -0.411386593204775
        ])
        time.sleep(3.0)
        self.capture_image("right")
        time.sleep(12.0)

        # Left
        self.move_arm([
            0.42609577779626956,
            0.6538980150581793,
            -0.8756588404408241,
            -1.6716867469879517,
            1.815057422373458
        ])
        time.sleep(3.0)
        self.capture_image("left")
        time.sleep(12.0)

        # Back
        self.move_arm([
            0.7416369754075611,
            0.6866735112936345,
            -0.78941063727839,
            -1.4135111876075732,
            -1.7329660238751148
        ])
        time.sleep(3.0)
        self.capture_image("back")
        time.sleep(12.0)

        # Front
        self.move_arm([
            0.8406754534899372,
            -0.18189024390243905,
            -2.301149976042166,
            -0.4799483648881239,
            -2.355188246097337
        ])
        time.sleep(3.0)
        self.capture_image("front")
        time.sleep(12.0)

        self.get_logger().info("🎯 Pick & rotate task finished.")

        # Enable policy bridge after pick & place finishes
        enable_msg = Bool()
        enable_msg.data = True
        self._policy_enable_pub.publish(enable_msg)
        self.get_logger().info("✅ Policy bridge enabled (published True to /policy_bridge/enable)")

        time.sleep(12.0)

        enable_msg.data = True
        self._policy_enable_pub.publish(enable_msg)
        self.get_logger().info("✅ Policy bridge disabled (published False to /policy_bridge/enable)")



def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    node.execute_task()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
