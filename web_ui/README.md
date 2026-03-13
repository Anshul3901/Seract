# Seract Web UI

Web-based user interface for the Seract robotic inspection system.

## Features

- **Live Camera Feed**: Real-time streaming of wrist camera feed
- **Captured Images**: Display images as they are captured from different views
- **Processing Status**: Visual indicator when AI is processing images
- **Decision Results**: Display inspection summary and decision
- **Logs Page**: Browse past inspection items with full details

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Build the ROS2 package:
```bash
cd ~/seract_orch_ws
colcon build --packages-select web_ui
source install/setup.bash
```

## Usage

### Option 1: Launch both nodes together
```bash
ros2 launch web_ui web_ui.launch.py
```

### Option 2: Run separately

Terminal 1 (ROS2 Bridge):
```bash
ros2 run web_ui web_ui_bridge
```

Terminal 2 (Flask Server):
```bash
ros2 run web_ui web_ui_server
```

Then open your browser to: http://localhost:5000

## Architecture

- **web_ui_bridge**: ROS2 node that subscribes to:
  - `/camera2/color/image_raw` - Camera feed
  - `/vision_detection_results` - Vision inspection results
  - Monitors `~/seract_orch_ws/scanned_pictures/` for new images

- **web_ui_server**: Flask web server with:
  - WebSocket support for real-time updates
  - REST API for past items
  - Serves HTML/CSS/JS frontend

## Pages

- **Dashboard** (`/`): Main inspection interface
- **Logs** (`/logs`): Browse past inspection items

