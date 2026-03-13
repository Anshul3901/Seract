#!/usr/bin/env python3
"""
ROS2 bridge node that subscribes to topics and provides data to the Flask web server.
Communicates via shared memory structures (thread-safe dicts/queues).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import threading
import time
from pathlib import Path
import os
from collections import deque
import base64
import numpy as np


class WebUIBridge(Node):
    """ROS2 node that bridges topics to Flask web UI."""
    
    def __init__(self):
        super().__init__('web_ui_bridge')
        
        # Shared state (thread-safe via locks)
        self.state_lock = threading.Lock()
        self.state = {
            'latest_camera_frame': None,  # base64 encoded JPEG
            'captured_images': {},  # {view_name: {'path': str, 'timestamp': float}}
            'processing_status': 'idle',  # 'idle', 'scanning', 'analyzing', 'complete'
            'current_view': None,
            'latest_decision': None,  # Full JSON result
            'latest_summary': None,
            'latest_decision_text': None,
            'system_stage': 'idle',  # 'idle', 'scanning', 'analyzing', 'accept', 'reject'
        }
        
        # Track recently captured images
        self.recent_images = deque(maxlen=10)
        
        # Image directory to watch
        self.image_dir = Path(os.path.expanduser('~/seract_orch_ws/scanned_pictures'))
        self.last_image_check = {}
        
        # Past items directory to watch for vision results
        self.past_items_dir = Path(os.path.expanduser('~/seract_orch_ws/past_items'))
        self.last_result_check = {}  # Track which result files we've processed
        self.last_item_number = 0
        
        # Decision.json file to monitor
        self.decision_file = Path(os.path.expanduser('~/seract_orch_ws/src/so101_motion/so101_motion/decision.json'))
        self.last_decision_seen = None  # Track last decision we've seen
        self.last_decision_file_mtime = 0.0
        
        self.bridge = CvBridge()
        
        # Track camera frame count for debugging
        self.camera_frame_count = 0
        self.last_camera_log_time = time.time()
        
        # Subscribers
        self.create_subscription(
            Image,
            '/object_debug_image',
            self.on_camera_image,
            10
        )
        
        self.create_subscription(
            String,
            '/vision_decision',
            self.on_vision_decision,
            10
        )
        
        self.get_logger().info("✅ Web UI Bridge node initialized")
        self.get_logger().info(f"📁 Watching image directory: {self.image_dir}")
        self.get_logger().info(f"📁 Watching past items directory: {self.past_items_dir}")
        self.get_logger().info(f"📹 Subscribing to camera topic: /object_debug_image")
        self.get_logger().info(f"📊 Subscribing to vision decision topic: /vision_decision")
        
        # Start image watcher timer
        self.create_timer(0.5, self.check_new_images)
        
        # Start vision results watcher timer
        self.create_timer(1.0, self.check_vision_results)  # Check every second
        
        # Start decision.json watcher timer
        self.create_timer(0.5, self.check_decision_file)  # Check every 0.5 seconds
        
        # Track last decision time to detect new cycles
        self.last_decision_time = 0.0
        self.decision_timeout = 30.0  # Reset after 30s of no activity
        
        # Shared state file for Flask server communication
        self.state_file = Path(os.path.expanduser('~/seract_orch_ws/.web_ui_state.json'))
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Timer to write state to file periodically
        self.create_timer(0.1, self.write_state_to_file)  # 10 Hz
    
    def on_camera_image(self, msg: Image):
        """Convert camera image to base64 JPEG for web streaming."""
        try:
            # /object_debug_image is published in bgr8 format by obj_bbox_node
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', cv_image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            jpeg_bytes = buffer.tobytes()
            b64_image = base64.b64encode(jpeg_bytes).decode('utf-8')
            
            with self.state_lock:
                self.state['latest_camera_frame'] = b64_image
            
            # Log periodically
            self.camera_frame_count += 1
            current_time = time.time()
            if current_time - self.last_camera_log_time > 5.0:
                self.get_logger().info(f"📹 Camera feed active: {self.camera_frame_count} frames received")
                self.last_camera_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def check_vision_results(self):
        """Check past_items directory for latest vision result JSON file."""
        if not self.past_items_dir.exists():
            return
        
        try:
            # Find all Item directories
            item_dirs = [d for d in self.past_items_dir.iterdir() if d.is_dir() and d.name.startswith('Item ')]
            if not item_dirs:
                return
            
            # Sort by item number (extract number from "Item X")
            def extract_item_num(item_dir):
                try:
                    return int(item_dir.name.replace('Item ', '').strip())
                except:
                    return 0
            
            # Get the latest item directory
            latest_item_dir = max(item_dirs, key=extract_item_num)
            latest_item_num = extract_item_num(latest_item_dir)
            
            # Always check latest item (even if same number) to get decision if decision.json is empty
            # Check if this is a new item OR if we need to update decision from past_items
            is_new_item = latest_item_num > self.last_item_number
            needs_decision_update = (not self.state.get('latest_decision_text') or 
                                    self.state.get('latest_decision_text') == 'None' or
                                    self.state.get('latest_decision_text') == '-')
            
            if not is_new_item and not needs_decision_update:
                return  # No new items and decision already set
            
            # Look for result JSON in vision_results folder
            results_dir = latest_item_dir / 'vision_results'
            if not results_dir.exists():
                return
            
            # Find result JSON files
            json_files = list(results_dir.glob('*_result.json'))
            if not json_files:
                return
            
            # Get the most recent result file
            latest_json = max(json_files, key=lambda p: p.stat().st_mtime)
            json_key = str(latest_json)
            
            # Check if we've already processed this file
            # But allow re-processing if decision is missing
            if json_key in self.last_result_check:
                # Only skip if we already have a decision
                if self.state.get('latest_decision_text') and self.state.get('latest_decision_text') not in ['None', '-', '']:
                    return
            
            # Read and process the result
            try:
                with open(latest_json, 'r') as f:
                    data = json.load(f)
                
                with self.state_lock:
                    # Check if this is a new cycle (new item_id)
                    current_item_id = self.state.get('latest_decision', {}).get('item_id')
                    new_item_id = data.get('item_id')
                    
                    if current_item_id and new_item_id != current_item_id:
                        # New cycle starting - reset captured images
                        self.state['captured_images'] = {}
                        self.state['processing_status'] = 'idle'
                        self.state['system_stage'] = 'idle'
                        self.get_logger().info(f"🔄 New cycle detected: {new_item_id}")
                    
                    # Always update decision from latest result (even if same item)
                    decision_text = data.get('result', 'Unknown')
                    if decision_text and decision_text != 'Unknown':
                        self.state['latest_decision'] = data
                        self.state['latest_summary'] = data.get('combined_summary', data.get('explanation', ''))
                        self.state['latest_decision_text'] = decision_text
                        self.state['processing_status'] = 'complete'
                        self.last_decision_time = time.time()
                        self.last_item_number = latest_item_num
                        
                        # Set system stage based on decision
                        decision_lower = decision_text.lower()
                        if 'restock' in decision_lower or 'resell' in decision_lower:
                            self.state['system_stage'] = 'accept'
                        elif 'recycle' in decision_lower or 'review' in decision_lower:
                            self.state['system_stage'] = 'reject'
                        else:
                            self.state['system_stage'] = 'complete'
                        
                        # Also update decision tracking
                        self.last_decision_seen = decision_text
                        
                        # Update captured images from result if available
                        if 'view_results' in data:
                            for view_name, view_data in data['view_results'].items():
                                img_path = view_data.get('image_path', '')
                                if img_path:
                                    # Convert to relative path if it's in past_items
                                    if 'past_items' in img_path:
                                        # Extract relative path
                                        parts = img_path.split('past_items/')
                                        if len(parts) > 1:
                                            rel_path = 'past_items/' + parts[1]
                                        else:
                                            rel_path = img_path
                                    else:
                                        rel_path = img_path
                                    
                                    self.state['captured_images'][view_name] = {
                                        'path': rel_path,
                                        'timestamp': time.time(),
                                        'view': view_name
                                    }
                
                self.last_result_check[json_key] = time.time()
                
                self.get_logger().info(
                    f"📊 Vision result loaded from {latest_item_dir.name}: {self.state['latest_decision_text']}"
                )
                # Force immediate state file write after vision result
                self.write_state_to_file()
                
                # Also update decision.json tracking if we got decision from past_items
                if self.state['latest_decision_text']:
                    self.last_decision_seen = self.state['latest_decision_text']
                
            except Exception as e:
                self.get_logger().error(f"Error reading vision result file {latest_json}: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                
        except Exception as e:
            self.get_logger().error(f"Error checking vision results: {e}")
    
    def check_new_images(self):
        """Check for newly captured images and update state."""
        # Reset status if decision was completed long ago (new cycle)
        current_time = time.time()
        with self.state_lock:
            if (self.state['processing_status'] == 'complete' and 
                self.last_decision_time > 0 and 
                current_time - self.last_decision_time > self.decision_timeout):
                # Reset for new cycle
                self.state['captured_images'] = {}
                self.state['processing_status'] = 'idle'
                self.state['system_stage'] = 'idle'
                self.state['current_view'] = None
                self.last_decision_time = 0.0
        
        if not self.image_dir.exists():
            return
            
        try:
            # Find all image files
            image_files = list(self.image_dir.glob('*.png')) + list(self.image_dir.glob('*.jpg'))
            
            for img_path in image_files:
                # Extract view name from filename (format: timestamp_view.ext)
                name_parts = img_path.stem.split('_')
                if len(name_parts) >= 2:
                    view_name = '_'.join(name_parts[1:])  # Handle multi-part view names
                else:
                    view_name = 'unknown'
                
                # Check if this is a new image
                mtime = img_path.stat().st_mtime
                key = str(img_path)
                
                if key not in self.last_image_check or self.last_image_check[key] < mtime:
                    self.last_image_check[key] = mtime
                    
                    with self.state_lock:
                        self.state['captured_images'][view_name] = {
                            'path': str(img_path),
                            'timestamp': mtime,
                            'view': view_name
                        }
                        self.state['current_view'] = view_name
                        # Update status based on number of images
                        num_images = len(self.state['captured_images'])
                        # Set stage to scanning when images are being captured
                        if num_images > 0 and num_images < 4:
                            self.state['processing_status'] = 'scanning'
                            self.state['system_stage'] = 'scanning'
                        elif num_images >= 4 and self.state['processing_status'] not in ['complete', 'analyzing']:
                            # All images captured, now analyzing
                            self.state['processing_status'] = 'analyzing'
                            self.state['system_stage'] = 'analyzing'
                    
                    self.recent_images.append({
                        'view': view_name,
                        'path': str(img_path),
                        'timestamp': mtime
                    })
                    
                    self.get_logger().info(f"📸 New image captured: {view_name} -> {img_path.name}")
        except Exception as e:
            self.get_logger().error(f"Error checking images: {e}")
    
    def get_state(self):
        """Thread-safe getter for current state."""
        with self.state_lock:
            return self.state.copy()
    
    def write_state_to_file(self):
        """Write current state to JSON file for Flask server to read."""
        try:
            state = self.get_state()
            # Don't write if state is empty/invalid
            if state:
                with open(self.state_file, 'w') as f:
                    json.dump(state, f)
        except Exception as e:
            self.get_logger().error(f"Error writing state file: {e}")
    
    def set_processing_status(self, status: str):
        """Set processing status."""
        with self.state_lock:
            self.state['processing_status'] = status
    
    def on_vision_decision(self, msg: String):
        """Handle vision decision from /vision_decision topic."""
        try:
            decision = msg.data.strip()
            if not decision:
                return
            
            with self.state_lock:
                # Update decision and stage
                decision_lower = decision.lower()
                if 'restock' in decision_lower or 'resell' in decision_lower:
                    self.state['system_stage'] = 'accept'
                    self.state['latest_decision_text'] = decision
                elif 'recycle' in decision_lower or 'review' in decision_lower:
                    self.state['system_stage'] = 'reject'
                    self.state['latest_decision_text'] = decision
                else:
                    self.state['system_stage'] = 'complete'
                    self.state['latest_decision_text'] = decision
                
                self.state['processing_status'] = 'complete'
                self.last_decision_time = time.time()
                self.last_decision_seen = decision
            
            self.get_logger().info(f"📋 Decision received from /vision_decision: {decision}")
            self.write_state_to_file()
            
        except Exception as e:
            self.get_logger().error(f"Error processing vision decision: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def check_decision_file(self):
        """Check decision.json file for latest decision."""
        # If file doesn't exist but we have a last decision, keep it in state
        if not self.decision_file.exists():
            # Preserve last decision even if file is erased
            if self.last_decision_seen:
                with self.state_lock:
                    if not self.state.get('latest_decision_text') or self.state['latest_decision_text'] != self.last_decision_seen:
                        decision_lower = self.last_decision_seen.lower()
                        if 'restock' in decision_lower or 'resell' in decision_lower:
                            self.state['system_stage'] = 'accept'
                        elif 'recycle' in decision_lower or 'review' in decision_lower:
                            self.state['system_stage'] = 'reject'
                        self.state['latest_decision_text'] = self.last_decision_seen
                        self.write_state_to_file()
            return
        
        try:
            # Check if file was modified
            mtime = self.decision_file.stat().st_mtime
            if mtime <= self.last_decision_file_mtime:
                return  # No change
            
            # Read the decision file
            with open(self.decision_file, 'r') as f:
                decision_data = json.load(f)
            
            decision = decision_data.get('decision', '').strip()
            
            # If file is empty but we have a last decision, preserve it
            if not decision and self.last_decision_seen:
                # File was erased but we keep the last decision
                with self.state_lock:
                    if not self.state.get('latest_decision_text') or self.state['latest_decision_text'] != self.last_decision_seen:
                        decision_lower = self.last_decision_seen.lower()
                        if 'restock' in decision_lower or 'resell' in decision_lower:
                            self.state['system_stage'] = 'accept'
                        elif 'recycle' in decision_lower or 'review' in decision_lower:
                            self.state['system_stage'] = 'reject'
                        self.state['latest_decision_text'] = self.last_decision_seen
                        self.state['processing_status'] = 'complete'
                        self.write_state_to_file()
                # Update mtime to avoid re-checking
                self.last_decision_file_mtime = mtime
                return
            
            # Update if we see a new decision (file was modified)
            if decision and decision != self.last_decision_seen:
                with self.state_lock:
                    # Determine stage based on decision
                    decision_lower = decision.lower()
                    if 'restock' in decision_lower or 'resell' in decision_lower:
                        self.state['system_stage'] = 'accept'
                        self.state['latest_decision_text'] = decision
                    elif 'recycle' in decision_lower or 'review' in decision_lower:
                        self.state['system_stage'] = 'reject'
                        self.state['latest_decision_text'] = decision
                    else:
                        self.state['system_stage'] = 'complete'
                        self.state['latest_decision_text'] = decision
                    
                    self.state['processing_status'] = 'complete'
                    self.last_decision_time = time.time()
                
                self.last_decision_seen = decision
                self.last_decision_file_mtime = mtime
                
                self.get_logger().info(f"📋 Decision updated from decision.json: {decision}")
                self.write_state_to_file()
            elif decision and decision == self.last_decision_seen:
                # File still has the same decision - update mtime but keep decision in state
                self.last_decision_file_mtime = mtime
                # Ensure state still has the decision even if file gets erased
                with self.state_lock:
                    if not self.state.get('latest_decision_text') or self.state['latest_decision_text'] != decision:
                        decision_lower = decision.lower()
                        if 'restock' in decision_lower or 'resell' in decision_lower:
                            self.state['system_stage'] = 'accept'
                        elif 'recycle' in decision_lower or 'review' in decision_lower:
                            self.state['system_stage'] = 'reject'
                        self.state['latest_decision_text'] = decision
                        self.write_state_to_file()
                
        except json.JSONDecodeError:
            # File might be empty or being written - ignore
            pass
        except Exception as e:
            self.get_logger().error(f"Error reading decision.json: {e}")


# Global instance for Flask access
_bridge_instance = None

def get_bridge_instance():
    """Get or create bridge instance."""
    global _bridge_instance
    if _bridge_instance is None:
        rclpy.init()
        _bridge_instance = WebUIBridge()
    return _bridge_instance

def main(args=None):
    rclpy.init(args=args)
    node = WebUIBridge()
    global _bridge_instance
    _bridge_instance = node
    
    # Store node globally for Flask access
    import web_ui.web_ui_server as server_module
    server_module.bridge_node = node
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

