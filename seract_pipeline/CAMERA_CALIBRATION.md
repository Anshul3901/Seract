# Camera Pose Calibration Guide

## Current Camera Pose

Based on the TF tree, the camera is currently at:
- **Translation**: `(0.4, 0.0, 0.6)` meters in `base_link`
- **Rotation**: Identity quaternion `(0, 0, 0, 1)` (camera looking down)

## Method 1: Check TF Tree (Current Method)

If the camera transform is already published, check it:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

This will show the current transform. The pipeline fallback now uses these values.

## Method 2: Publish Static Transform

To make the transform always available, publish it as a static transform:

### Option A: Use the provided script

```bash
cd ~/seract_ws
source install/setup.bash
source /opt/ros/humble/setup.bash
./src/seract_pipeline/scripts/publish_camera_tf.sh
```

### Option B: Manual command

```bash
ros2 run tf2_ros static_transform_publisher \
  0.4 0.0 0.6 \
  0.0 0.0 0.0 1.0 \
  base_link camera_link
```

### Option C: Add to launch file

Add this to your launch file:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_camera_tf',
    arguments=[
        '0.4', '0.0', '0.6',  # x, y, z translation
        '0.0', '0.0', '0.0', '1.0',  # quaternion (identity)
        'base_link',
        'camera_link'
    ],
)
```

## Method 3: Calibrate Using Calibration Helper

If you need to recalibrate the camera pose:

1. **Run the calibration helper**:
   ```bash
   ros2 run seract_pipeline calib_helper
   ```

2. **Get a grasp pose from the policy**:
   - Run the policy server and get a grasp pose response
   - Note the position values in `camera_link` frame

3. **Place an object at a known position**:
   - Place a flat object (like a puck) at a known XY position on the table
   - Measure or know the table height in `base_link`

4. **Enter values when prompted**:
   - Enter the grasp pose from the policy (in `camera_link`)
   - Enter the table XY position (in `base_link`)

5. **The tool will output**:
   - A `static_transform_publisher` command
   - Launch file XML snippet

## Method 4: Manual Measurement

If you can physically measure the camera position:

1. **Measure camera position relative to base**:
   - X: Distance forward from base
   - Y: Distance left/right from base
   - Z: Height above base

2. **Measure camera orientation**:
   - For top-down camera: typically identity quaternion `(0, 0, 0, 1)`
   - For angled camera: measure roll, pitch, yaw and convert to quaternion

3. **Publish the transform** using Method 2 above

## Updating the Fallback Transform

If you change the camera pose, update the fallback transform in:
- `seract_pipeline/seract_pipeline/pipeline.py`
- Look for the `transform_to_base_link` method
- Update the `camera_x_base`, `camera_y_base`, `camera_z_base` values

## Verification

After publishing the transform, verify it:

```bash
# Check if transform exists
ros2 run tf2_ros tf2_echo base_link camera_link

# List all frames
ros2 run tf2_ros tf2_monitor
```

The pipeline will automatically use the TF transform if available, or fall back to the hardcoded values if not.

