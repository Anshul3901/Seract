# Seract Policy Control - Joint Jogger

A ROS2 package for controlling the SO101 robot arm with joint-level jogging capabilities. This package provides a flexible joint jogger that can move single or multiple joints simultaneously with sinusoidal motion patterns.

## Overview

The `joint_jogger` node publishes joint trajectory commands to the robot's `JointTrajectoryController`, enabling smooth, controlled movement of individual or multiple joints. This is useful for:
- Testing robot hardware and controllers
- Calibrating joint movements
- Demonstrating robot capabilities
- Debugging control issues

## Key Features

- **Single or Multi-Joint Control**: Jog one joint or multiple joints simultaneously
- **Sinusoidal Motion**: Smooth, predictable movement patterns
- **Phase Offsets**: Multi-joint movements use phase offsets for smoother motion
- **Safety Limits**: Configurable amplitude, frequency, and max step size
- **Auto-Discovery**: Automatically discovers joint names from `/joint_states`

## What It Does

The `joint_jogger` node:
1. Subscribes to `/joint_states` to get current joint positions
2. Calculates desired positions using sinusoidal motion with configurable parameters
3. Publishes `JointTrajectory` messages to `/so_100_arm_controller/joint_trajectory`
4. Updates at 10 Hz with configurable motion parameters

## Key Learnings

### 1. Hardware Integration
- The SO101 robot uses Feetech servos controlled via ROS2 control
- Hardware driver (`/so_arm_100_driver`) must be running before controllers
- All 6 hardware interfaces must be claimed (5 arm joints + 1 gripper)
- The `JointTrajectoryController` requires specific message format:
  - All 5 arm joints must be included (no Gripper)
  - Last point must have zero velocity
  - Valid timestamp and `time_from_start` required

### 2. Multi-Joint Control
- **Phase Offsets**: When jogging multiple joints, phase offsets spread the motion evenly across the cycle for smoother overall movement
- **Special Mode**: Using `jog_joint_index:=999` triggers 3-joint mode (Shoulder_Rotation, Shoulder_Pitch, Elbow)
- **Parameter Priority**: `jog_joint_indices` parameter takes precedence over `jog_joint_index`

### 3. Movement Parameters
- **Amplitude**: Peak movement range (radians). Larger = more visible movement
- **Frequency**: Speed of oscillation (Hz). Higher = faster cycles
- **Max Step**: Maximum change per update (radians). Limits how fast joints can move per update
- **Command DT**: Time horizon for trajectory points (0.25 seconds default)

### 4. Parameter Optimization
Based on testing:
- **Too Small Movement** (< 5 deg): Increase amplitude (0.20 → 0.25-0.30 rad) and max_step (0.15 → 0.20-0.25 rad)
- **Good Movement** (5-15 deg): Keep amplitude=0.20, frequency=0.30, max_step=0.15
- **Too Fast/Jerky**: Decrease frequency (0.30 → 0.20-0.25 Hz) and max_step (0.15 → 0.10-0.12 rad)

### 5. Build and Installation Issues
- Python packages need proper installation via `colcon build`
- Code changes require copying to install directory or rebuilding
- Package metadata issues can prevent node from starting
- Workaround: Copy updated code directly to `install/seract_policy_control/lib/seract_policy_control/`

## File Structure and Roles

```
seract_policy_control/
├── README.md                          # This file
├── package.xml                        # ROS2 package metadata
├── setup.py                           # Python package setup (includes build fix)
├── setup.cfg                          # Setuptools configuration
├── resource/
│   └── seract_policy_control          # Package resource marker
├── seract_policy_control/
│   ├── __init__.py                    # Package initialization
│   ├── joint_jogger.py                # Main joint jogger node implementation
│   ├── lerobot_policy_controller.py   # LeRobot policy controller (future use)
│   └── safety.py                      # Safety module (joint limits, step limiting, timeout)
├── launch/
│   └── joint_jogger.launch.py         # Launch file (with array param support)
└── config/
    └── policy_control.yaml            # Configuration file (for lerobot controller)
```

### Key Files

#### `joint_jogger.py`
**Role**: Main implementation of the joint jogger node

**Key Components**:
- `JointJogger` class: ROS2 node that publishes joint trajectories
- `on_js()`: Callback for `/joint_states` subscription (auto-discovers joints)
- `tick()`: Timer callback that calculates and publishes trajectory commands
- `clamp()`: Ensures joint positions stay within limits

**Key Parameters**:
- `jog_joint_index`: Single joint to jog (0-4) or 999 for 3-joint mode
- `jog_joint_indices`: Array of joint indices to jog (takes precedence)
- `jog_amplitude`: Peak movement range in radians
- `jog_frequency`: Oscillation frequency in Hz
- `max_step_rad`: Maximum change per update in radians

**Multi-Joint Logic**:
```python
# Special mode: jog_joint_index:=999 triggers 3-joint mode
if self.jog_idx == 999:
    joints_to_jog = [0, 1, 2]  # Shoulder_Rotation, Shoulder_Pitch, Elbow

# Phase offsets for smoother motion
phase_offset = (2.0 * math.pi * i) / len(joints_to_jog)
target_offset = self.jog_amp * math.sin(2.0 * math.pi * self.jog_freq * t + phase_offset)
```

#### `joint_jogger.launch.py`
**Role**: Launch file for starting the joint jogger node

**Key Features**:
- Sets environment variables (RMW_FASTRTPS_USE_SHM=0, ROS_DOMAIN_ID=0)
- Unsets conda environment variables to avoid conflicts
- Declares launch arguments for all parameters
- Uses `OpaqueFunction` to properly handle array parameters (`jog_joint_indices`)
- Parses `jog_joint_indices` from string format (e.g., `"[0,1,2]"`) to Python list

**Usage**:
```bash
# Using jog_joint_index (single joint or special mode 999 for 3 joints)
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_index:=999 \
  jog_amplitude:=0.20 \
  jog_frequency:=0.30 \
  max_step_rad:=0.15

# Using jog_joint_indices (custom joint selection)
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_indices:="[0,1,2]" \
  jog_amplitude:=0.20 \
  jog_frequency:=0.30 \
  max_step_rad:=0.15
```

**Note**: The launch file uses `OpaqueFunction` to properly parse array parameters from command-line strings, avoiding ROS2 launch type errors.

#### `setup.py`
**Role**: Python package configuration

**Key Components**:
- Defines console script entry point: `joint_jogger = seract_policy_control.joint_jogger:main`
- Installs launch files and config files
- Specifies package dependencies

## Prerequisites

1. **ROS2 Humble** installed and sourced
2. **Hardware driver running**: `/so_arm_100_driver` must be active
3. **Controllers active**: `so_100_arm_controller` must be running
4. **Hardware interfaces claimed**: All 6 interfaces must be claimed

## Installation

```bash
cd ~/seract_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select seract_policy_control
source install/setup.bash
```

## Usage

### Basic Usage - Single Joint

Jog a single joint (e.g., Shoulder_Rotation = joint 0):

```bash
cd ~/seract_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0
export ROS_DOMAIN_ID=0

ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_index:=0 \
  jog_amplitude:=0.15 \
  jog_frequency:=0.25 \
  max_step_rad:=0.10
```

### Multi-Joint Mode - 3 Joints

Jog 3 joints simultaneously (Shoulder_Rotation, Shoulder_Pitch, Elbow):

```bash
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_index:=999 \
  jog_amplitude:=0.20 \
  jog_frequency:=0.30 \
  max_step_rad:=0.15
```

### Custom Joint Selection

Jog specific joints using `jog_joint_indices`:

```bash
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_indices:="[0,1,2]" \
  jog_amplitude:=0.20 \
  jog_frequency:=0.30 \
  max_step_rad:=0.15
```

### Recommended Parameter Sets

**For Good Visible Movement**:
```bash
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_index:=999 \
  jog_amplitude:=0.20 \
  jog_frequency:=0.30 \
  max_step_rad:=0.15
```

**For Excellent Visible Movement**:
```bash
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_index:=999 \
  jog_amplitude:=0.25 \
  jog_frequency:=0.35 \
  max_step_rad:=0.20
```

**For Slow, Smooth Movement**:
```bash
ros2 launch seract_policy_control joint_jogger.launch.py \
  jog_joint_index:=999 \
  jog_amplitude:=0.15 \
  jog_frequency:=0.20 \
  max_step_rad:=0.10
```

## Joint Mapping

The SO101 robot has 5 arm joints (Gripper is separate):

| Index | Joint Name        | Description           |
|-------|-------------------|-----------------------|
| 0     | Shoulder_Rotation | Base rotation joint   |
| 1     | Shoulder_Pitch    | Shoulder pitch joint  |
| 2     | Elbow             | Elbow joint           |
| 3     | Wrist_Pitch       | Wrist pitch joint     |
| 4     | Wrist_Roll        | Wrist roll joint      |

**Special Mode**: `jog_joint_index:=999` jogs joints 0, 1, and 2 simultaneously.

## Monitoring Movement

### Check if Node is Running
```bash
ros2 node list | grep joint_jogger
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Monitor Trajectory Commands
```bash
ros2 topic echo /so_100_arm_controller/joint_trajectory
```

### Check Controller Status
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Troubleshooting

### Node Not Starting
- **Issue**: `PackageNotFoundError: No package metadata was found`
- **Solution**: Rebuild the package or copy updated code to install directory:
  ```bash
  # Option 1: Rebuild (recommended)
  colcon build --packages-select seract_policy_control
  source install/setup.bash
  
  # Option 2: Copy directly (faster for testing)
  mkdir -p install/seract_policy_control/lib/seract_policy_control
  cp src/seract_policy_control/seract_policy_control/*.py \
     install/seract_policy_control/lib/seract_policy_control/
  ```

### Launch File Error: "Expected a non-empty sequence"
- **Issue**: Error when using `jog_joint_indices` parameter
- **Solution**: The launch file has been fixed to properly handle array parameters. Rebuild the package:
  ```bash
  colcon build --packages-select seract_policy_control
  source install/setup.bash
  ```
  Or copy the launch file:
  ```bash
  cp src/seract_policy_control/launch/joint_jogger.launch.py \
     install/seract_policy_control/share/seract_policy_control/launch/joint_jogger.launch.py
  ```

### Movement Too Small
- **Issue**: Joints moving < 5 degrees
- **Solution**: Increase `jog_amplitude` (0.20 → 0.25-0.30) and `max_step_rad` (0.15 → 0.20-0.25)

### Movement Too Fast/Jerky
- **Issue**: Movement is jerky or too fast
- **Solution**: Decrease `jog_frequency` (0.30 → 0.20-0.25) and `max_step_rad` (0.15 → 0.10-0.12)

### Controller Rejects Commands
- **Issue**: "Joints on incoming trajectory don't match"
- **Solution**: Ensure Gripper is filtered out (code does this automatically)

### No Movement
- **Issue**: Commands published but no movement
- **Check**:
  1. Hardware driver is running: `ros2 node list | grep driver`
  2. Controllers are active: `ros2 control list_controllers`
  3. Hardware interfaces are claimed: `ros2 control list_hardware_interfaces`
  4. Joint states are publishing: `ros2 topic echo /joint_states --once`

## Code Updates

After modifying code files, you need to either:

1. **Rebuild the package** (recommended):
   ```bash
   colcon build --packages-select seract_policy_control
   source install/setup.bash
   ```

2. **Or copy directly** (faster for testing):
   ```bash
   mkdir -p install/seract_policy_control/lib/seract_policy_control
   cp src/seract_policy_control/seract_policy_control/*.py \
      install/seract_policy_control/lib/seract_policy_control/
   # Also copy launch files if modified
   cp src/seract_policy_control/launch/*.launch.py \
      install/seract_policy_control/share/seract_policy_control/launch/
   ```

### Build Fix

The `setup.py` includes a workaround for colcon build issues. It automatically filters out unknown command-line options that colcon passes but setuptools doesn't recognize (like `--uninstall`, `--build-directory`, `--build-base`, etc.). This allows the package to build successfully without errors.

## Safety Module

The `safety.py` module provides safety checks for joint control:

- **Joint Limit Checking**: `joint_limits(q)` - Clips joint positions to stay within `qmin` and `qmax`
- **Step Size Limiting**: `step_limit(q_curr, q_cmd)` - Limits change per update to `max_step`
- **Joint State Timeout**: `mark_js()`, `ok()` - Monitors if joint states are received within timeout
- **Emergency Stop**: `estop` flag - Immediately stops operation when set

**Usage Example**:
```python
from seract_policy_control.safety import Safety

safety = Safety(
    qmin=[-3.14, -1.57, 0.0, -1.57, -3.14],
    qmax=[3.14, 1.57, 3.14, 1.57, 3.14],
    max_step=0.15,
    js_timeout=1.0
)

safety.mark_js()  # Call when joint state received
if safety.ok():
    q_safe = safety.step_limit(q_current, q_command)
    q_safe = safety.joint_limits(q_safe)
```

## Future Improvements

- [ ] Integrate `safety.py` module into `joint_jogger.py`
- [ ] Add velocity and acceleration limits
- [ ] Support for custom motion patterns (not just sinusoidal)
- [ ] GUI for parameter adjustment
- [ ] Save/load parameter presets
- [ ] Real-time parameter adjustment via ROS2 parameters
- [ ] Integration with MoveIt for collision checking

## Related Packages

- `so_100_arm`: Hardware driver for SO101 robot
- `so101_moveit_config`: MoveIt configuration for SO101
- `seract_primitives`: High-level primitive actions (pick, place, etc.)

## License

MIT

## Authors

Created for SO101 robot control and testing.
