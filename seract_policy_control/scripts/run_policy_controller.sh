#!/bin/bash
# Wrapper script to run lerobot_policy_controller with correct environment
# This ensures conda is deactivated and system libstdc++ is used

set -e

# Remove conda from environment if active (without calling conda deactivate)
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "WARNING: Conda environment '$CONDA_DEFAULT_ENV' is active."
    echo "Removing conda from PATH to avoid GLIBCXX version mismatch..."
    
    # Remove conda paths from PATH
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    
    # Unset conda-related variables
    unset CONDA_DEFAULT_ENV
    unset CONDA_PREFIX
    unset CONDA_PROMPT_MODIFIER
    unset CONDA_PYTHON_EXE
    unset CONDA_SHLVL
fi

# Ensure system libstdc++ is used (prioritize system libraries)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}

# Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Source workspace
if [ -f ~/seract_ws/install/setup.bash ]; then
    source ~/seract_ws/install/setup.bash
else
    echo "ERROR: Workspace not found at ~/seract_ws"
    exit 1
fi

# Set ROS2 environment variables
export RMW_FASTRTPS_USE_SHM=0
export ROS_DOMAIN_ID=0

# Run the launch file with all arguments passed through
exec ros2 launch seract_policy_control run_policy_or_moveit.launch.py "$@"

