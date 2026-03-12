#!/usr/bin/env python3
"""
Compute camera pose from policy response and known object position.
"""

import sys

def compute_camera_pose():
    print("=== Compute Camera Pose from Policy Response ===\n")
    
    print("The policy returned a grasp pose in camera_link frame:")
    print("  position: (0.351, 0.052, -0.419)")
    print("  orientation: (0.424, 0.496, -0.022, 1.0)\n")
    
    print("To compute the camera pose, we need:")
    print("1. The grasp pose in camera_link (from policy) ✓")
    print("2. Where that object actually is in base_link (you need to measure)\n")
    
    print("Please place an object at a known position on the table.")
    print("Then run the policy to get a grasp pose for that object.\n")
    
    try:
        print("Enter the grasp pose from policy (in camera_link frame):")
        cam_x = float(input("  X (meters): "))
        cam_y = float(input("  Y (meters): "))
        cam_z = float(input("  Z (meters): "))
        
        print("\nEnter where that object actually is (in base_link frame):")
        base_x = float(input("  X (meters): "))
        base_y = float(input("  Y (meters): "))
        base_z = float(input("  Z (meters): "))
        
        # For top-down camera, z points down, so:
        # base_z = camera_z_base + camera_z (camera z is negative)
        # base_x = camera_x_base + camera_x
        # base_y = camera_y_base + camera_y
        
        camera_x_base = base_x - cam_x
        camera_y_base = base_y - cam_y
        camera_z_base = base_z - cam_z  # camera z is negative, so we subtract
        
        print("\n=== Camera Pose Computed ===")
        print(f"Translation: ({camera_x_base:.6f}, {camera_y_base:.6f}, {camera_z_base:.6f})")
        print("Rotation: (0.000000, 0.000000, 0.000000, 1.000000)  # identity for top-down\n")
        
        print("To publish this transform, run:")
        print(f"ros2 run tf2_ros static_transform_publisher \\")
        print(f"  {camera_x_base:.6f} {camera_y_base:.6f} {camera_z_base:.6f} \\")
        print(f"  0.0 0.0 0.0 1.0 \\")
        print(f"  base_link camera_link")
        
    except (ValueError, EOFError, KeyboardInterrupt):
        print("\nCancelled or invalid input.")
        print("\nQuick reference:")
        print("  Current fallback: (0.4, 0.0, 0.6)")
        print("  Calibration helper: (0.4, 0.0, 0.48)")
        print("\nTo use the current fallback, run:")
        print("ros2 run tf2_ros static_transform_publisher \\")
        print("  0.4 0.0 0.6 \\")
        print("  0.0 0.0 0.0 1.0 \\")
        print("  base_link camera_link")

if __name__ == '__main__':
    compute_camera_pose()

