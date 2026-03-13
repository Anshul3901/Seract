#!/usr/bin/env python3
"""
Flask web server for Seract inspection UI.
Serves the dashboard and logs pages, provides WebSocket for real-time updates.
"""

from flask import Flask, render_template, jsonify, send_from_directory, Response
from flask_socketio import SocketIO, emit
import threading
import time
import json
from pathlib import Path
import os
import glob

# Global bridge node (set by bridge node)
bridge_node = None

# State file path (shared with bridge node)
STATE_FILE = Path(os.path.expanduser('~/seract_orch_ws/.web_ui_state.json'))

def get_state_from_file():
    """Read state from shared JSON file."""
    try:
        if STATE_FILE.exists():
            with open(STATE_FILE, 'r') as f:
                return json.load(f)
    except Exception as e:
        print(f"Error reading state file: {e}")
    return {
        'latest_camera_frame': None,
        'captured_images': {},
        'processing_status': 'idle',
        'current_view': None,
        'latest_decision': None,
        'latest_summary': None,
        'latest_decision_text': None,
    }

# Determine template and static folders
# When installed via ROS2, files are in share/web_ui/
# When running from source, files are in ../templates and ../static
_script_dir = os.path.dirname(os.path.abspath(__file__))
_template_dir = os.path.join(_script_dir, '..', 'templates')
_static_dir = os.path.join(_script_dir, '..', 'static')

# Check if running from installed location
if not os.path.exists(_template_dir):
    # Try installed location
    try:
        import ament_index_python
        share_dir = ament_index_python.get_package_share_directory('web_ui')
        _template_dir = os.path.join(share_dir, 'templates')
        _static_dir = os.path.join(share_dir, 'static')
    except:
        # Fallback: use script directory
        _template_dir = os.path.join(_script_dir, '..', 'templates')
        _static_dir = os.path.join(_script_dir, '..', 'static')

app = Flask(__name__, 
            template_folder=_template_dir,
            static_folder=_static_dir)
app.config['SECRET_KEY'] = 'seract_web_ui_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Paths
PAST_ITEMS_DIR = Path(os.path.expanduser('~/seract_orch_ws/past_items'))
SCANNED_PICTURES_DIR = Path(os.path.expanduser('~/seract_orch_ws/scanned_pictures'))


@app.route('/')
def index():
    """Main dashboard page."""
    return render_template('dashboard.html')


@app.route('/logs')
def logs():
    """Logs/history page."""
    return render_template('logs.html')


@app.route('/api/state')
def api_state():
    """Get current state from bridge node."""
    state = get_state_from_file()
    return jsonify(state)


def extract_item_number(item_name):
    """Extract numeric item number from 'Item X' format for proper sorting."""
    try:
        # Remove 'Item ' prefix and convert to int
        num_str = item_name.replace('Item ', '').strip()
        return int(num_str)
    except (ValueError, AttributeError):
        # If not a number, return a large number to sort to end
        return 999999

@app.route('/api/past_items')
def api_past_items():
    """Get list of past items."""
    if not PAST_ITEMS_DIR.exists():
        return jsonify([])
    
    items = []
    # Sort by item number (numeric) instead of alphabetical
    item_dirs = [d for d in PAST_ITEMS_DIR.iterdir() if d.is_dir()]
    item_dirs_sorted = sorted(item_dirs, key=lambda x: extract_item_number(x.name), reverse=True)
    
    for item_dir in item_dirs_sorted:
        # Extract item number
        item_num = item_dir.name.replace('Item ', '')
        
        # Find result JSON
        results_dir = item_dir / 'vision_results'
        result_json = None
        if results_dir.exists():
            json_files = list(results_dir.glob('*_result.json'))
            if json_files:
                try:
                    with open(json_files[0], 'r') as f:
                        result_json = json.load(f)
                except Exception as e:
                    print(f"Error reading {json_files[0]}: {e}")
        
        # Count images
        images_dir = item_dir / 'images'
        image_count = 0
        if images_dir.exists():
            image_count = len(list(images_dir.glob('*.png')) + list(images_dir.glob('*.jpg')))
        
        items.append({
            'item_number': item_num,
            'item_name': item_dir.name,
            'item_path': str(item_dir),
            'timestamp': result_json.get('timestamp', '') if result_json else '',
            'decision': result_json.get('result', 'Unknown') if result_json else 'Unknown',
            'object_type': result_json.get('object_identification', {}).get('object_type', 'Unknown') if result_json and isinstance(result_json.get('object_identification'), dict) else 'Unknown',
            'image_count': image_count,
            'summary': result_json.get('combined_summary', result_json.get('explanation', '')) if result_json else ''
        })
    
    return jsonify(items)


@app.route('/api/past_item/<item_name>')
def api_past_item(item_name):
    """Get details for a specific past item."""
    item_dir = PAST_ITEMS_DIR / item_name
    if not item_dir.exists():
        return jsonify({'error': 'Item not found'}), 404
    
    # Load result JSON
    results_dir = item_dir / 'vision_results'
    result_json = None
    if results_dir.exists():
        json_files = list(results_dir.glob('*_result.json'))
        if json_files:
            try:
                with open(json_files[0], 'r') as f:
                    result_json = json.load(f)
            except Exception as e:
                return jsonify({'error': f'Error reading result: {e}'}), 500
    
    # Get images
    images_dir = item_dir / 'images'
    images = []
    if images_dir.exists():
        for img_path in sorted(images_dir.glob('*.png')) + sorted(images_dir.glob('*.jpg')):
            # Extract view name
            name_parts = img_path.stem.split('_')
            view_name = '_'.join(name_parts[1:]) if len(name_parts) >= 2 else 'unknown'
            # Use relative path from past_items directory for easier serving
            rel_path = img_path.relative_to(PAST_ITEMS_DIR)
            images.append({
                'view': view_name,
                'path': str(rel_path),  # Relative path like "Item 1/images/image.png"
                'filename': img_path.name,
                'full_path': str(img_path)  # Keep full path as backup
            })
    
    return jsonify({
        'item_name': item_name,
        'result': result_json,
        'images': images
    })


@app.route('/api/image/<path:image_path>')
def api_image(image_path):
    """Serve image files."""
    try:
        # image_path can be:
        # 1. Relative path like "Item 1/images/image.png" (from past_items)
        # 2. Absolute path like "/home/.../past_items/Item 1/images/image.png"
        # 3. Just filename if in scanned_pictures
        
        image_file = None
        
        # First, try as relative path from past_items
        past_path = PAST_ITEMS_DIR / image_path
        if past_path.exists() and past_path.is_file():
            image_file = past_path
        
        # If not found, try as absolute path
        if image_file is None:
            abs_path = Path(image_path)
            if abs_path.is_absolute() and abs_path.exists() and abs_path.is_file():
                # Security check: must be in allowed directories
                scanned_normalized = str(SCANNED_PICTURES_DIR.resolve())
                past_normalized = str(PAST_ITEMS_DIR.resolve())
                image_normalized = str(abs_path.resolve())
                
                if image_normalized.startswith(scanned_normalized) or image_normalized.startswith(past_normalized):
                    image_file = abs_path
        
        # If still not found, try scanned_pictures
        if image_file is None:
            scanned_path = SCANNED_PICTURES_DIR / image_path
            if scanned_path.exists() and scanned_path.is_file():
                image_file = scanned_path
        
        # If we found the file, serve it
        if image_file and image_file.exists() and image_file.is_file():
            # Final security check
            scanned_normalized = str(SCANNED_PICTURES_DIR.resolve())
            past_normalized = str(PAST_ITEMS_DIR.resolve())
            image_normalized = str(image_file.resolve())
            
            if image_normalized.startswith(scanned_normalized) or image_normalized.startswith(past_normalized):
                return send_from_directory(str(image_file.parent), image_file.name)
        
        return jsonify({'error': f'Image not found: {image_path}'}), 404
    except Exception as e:
        import traceback
        print(f"Error serving image {image_path}: {e}")
        print(traceback.format_exc())
        return jsonify({'error': f'Error serving image: {str(e)}'}), 500


@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection."""
    print('Client connected')
    emit('status', {'msg': 'Connected to Seract Web UI'})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection."""
    print('Client disconnected')


def broadcast_updates():
    """Background thread to broadcast state updates."""
    while True:
        state = get_state_from_file()
        socketio.emit('state_update', state)
        time.sleep(0.1)  # 10 Hz update rate


# Start background thread for updates
update_thread = threading.Thread(target=broadcast_updates, daemon=True)
update_thread.start()


def main():
    """Run Flask server."""
    print("🚀 Starting Seract Web UI Server...")
    print(f"📁 Past items directory: {PAST_ITEMS_DIR}")
    print(f"📁 Scanned pictures directory: {SCANNED_PICTURES_DIR}")
    print(f"📄 State file: {STATE_FILE}")
    print(f"🌐 Web UI available at: http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()

