#!/bin/bash
# Script to find camera pose

echo "=== Finding Camera Pose ==="
echo ""

# Check if camera_link exists in TF tree
echo "1. Checking TF tree for camera_link..."
timeout 3 ros2 run tf2_ros tf2_echo base_link camera_link 2>&1 | grep -A 5 "Translation\|Rotation" || echo "   ❌ camera_link not found in TF tree"
echo ""

# Check for static transforms
echo "2. Checking for static transforms..."
timeout 2 ros2 topic echo /tf_static --once 2>&1 | grep -A 3 camera_link || echo "   ❌ No static transform found"
echo ""

# Check camera node frame_id
echo "3. Checking camera node frame_id..."
ros2 param get /usb_camera_node frame_id 2>&1 | grep -v "RTPS_TRANSPORT_SHM" || echo "   ❌ Camera node not running"
echo ""

echo "=== Options to Find Camera Pose ==="
echo ""
echo "Option A: Use calibration helper (recommended)"
echo "  ros2 run seract_pipeline calib_helper"
echo ""
echo "Option B: Publish static transform manually"
echo "  ros2 run tf2_ros static_transform_publisher \\"
echo "    0.4 0.0 0.6 \\"
echo "    0.0 0.0 0.0 1.0 \\"
echo "    base_link camera_link"
echo ""
echo "Option C: Use the provided script"
echo "  ./src/seract_pipeline/scripts/publish_camera_tf.sh"
echo ""

