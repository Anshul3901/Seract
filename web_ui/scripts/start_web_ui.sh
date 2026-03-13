#!/bin/bash
# Startup script for Web UI

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo "🚀 Starting Seract Web UI..."
echo "📁 Workspace: $WS_DIR"

# Source ROS2 setup
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Install Python dependencies if needed
if ! python3 -c "import flask" 2>/dev/null; then
    echo "📦 Installing Python dependencies..."
    pip3 install flask flask-socketio python-socketio eventlet --user
fi

# Start bridge node in background
echo "🔌 Starting ROS2 bridge node..."
ros2 run web_ui web_ui_bridge &
BRIDGE_PID=$!

# Wait a moment for bridge to initialize
sleep 2

# Start Flask server
echo "🌐 Starting Flask web server..."
ros2 run web_ui web_ui_server

# Cleanup on exit
trap "kill $BRIDGE_PID 2>/dev/null || true" EXIT

