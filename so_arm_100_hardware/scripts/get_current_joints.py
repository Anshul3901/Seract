#!/usr/bin/env python3
"""
Get current joint positions from /joint_states topic.
"""
import rclpy
from sensor_msgs.msg import JointState
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('get_current_joints')
    
    joint_states = None
    
    def callback(msg):
        nonlocal joint_states
        joint_states = msg
    
    sub = node.create_subscription(JointState, '/joint_states', callback, 10)
    
    # Wait for message
    import time
    timeout = 2.0
    start = time.time()
    while joint_states is None and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if joint_states is None:
        print("ERROR: No joint_states received. Make sure hardware is running.")
        rclpy.shutdown()
        sys.exit(1)
    
    # Find arm joints
    arm_joints = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
    gripper_joint = 'Gripper'
    
    print("\n=== Current Joint Positions ===")
    print(f"\nArm Joints:")
    for joint_name in arm_joints:
        try:
            idx = joint_states.name.index(joint_name)
            pos = joint_states.position[idx]
            print(f"  {joint_name:20s}: {pos:8.4f} rad ({pos*180/3.14159:7.2f} deg)")
        except ValueError:
            print(f"  {joint_name:20s}: NOT FOUND")
    
    print(f"\nGripper:")
    try:
        idx = joint_states.name.index(gripper_joint)
        pos = joint_states.position[idx]
        print(f"  {gripper_joint:20s}: {pos:8.4f} rad ({pos*180/3.14159:7.2f} deg)")
    except ValueError:
        print(f"  {gripper_joint:20s}: NOT FOUND")
    
    # Print as array for easy copy-paste
    print(f"\nAs array (for reset_to_mid.py or move_arm):")
    positions = []
    for joint_name in arm_joints:
        try:
            idx = joint_states.name.index(joint_name)
            positions.append(joint_states.position[idx])
        except ValueError:
            positions.append(0.0)
    print(f"  [{', '.join(f'{p:.4f}' for p in positions)}]")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

