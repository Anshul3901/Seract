"""
ROS2 node that monitors image directory and runs vision inspection pipeline.
Processes images automatically when they are saved and outputs JSON results.
"""

import os
import json
import time
import threading
import shutil
import re
from pathlib import Path
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Try to import watchdog
try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler, FileCreatedEvent
    WATCHDOG_AVAILABLE = True
except ImportError:
    WATCHDOG_AVAILABLE = False
    Observer = None
    FileSystemEventHandler = None

from .detection_pipeline import run_detectors_on_image, openai_detection_multi


class ImageFileHandler(FileSystemEventHandler):
    """Handler for file system events - processes new image files."""
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.processed_files = set()
        self.processing_lock = threading.Lock()
    
    def on_created(self, event):
        """Called when a new file is created."""
        if event.is_directory:
            return
        
        file_path = Path(event.src_path)
        
        # Only process image files
        if file_path.suffix.lower() not in ['.png', '.jpg', '.jpeg']:
            return
        
        # Skip temporary compressed files (they're in /tmp, but just in case)
        if 'temp_compressed' in file_path.name or 'openai_compressed' in file_path.name:
            return
        
        # Check if file exists (might be a false positive from watchdog)
        if not file_path.exists():
            self.node.get_logger().debug(f"File {file_path.name} detected but doesn't exist yet, skipping")
            return
        
        # Ensure vision_results folder exists (create on-demand when first image detected)
        self.node.output_directory.mkdir(parents=True, exist_ok=True)
        
        # Skip if already processed
        with self.processing_lock:
            if str(file_path) in self.processed_files:
                self.node.get_logger().debug(f"File {file_path.name} already processed, skipping")
                return
            self.processed_files.add(str(file_path))
            self.node.get_logger().info(f"📁 New file detected: {file_path.name} (total tracked: {len(self.processed_files)})")
        
        # Wait a bit for file to be fully written
        time.sleep(0.5)
        
        # Double-check file still exists after wait
        if not file_path.exists():
            self.node.get_logger().warn(f"File {file_path.name} disappeared after wait, skipping")
            with self.processing_lock:
                self.processed_files.discard(str(file_path))
            return
        
        # Process the image
        self.node.process_image_async(file_path)


class VisionInspectionNode(Node):
    """ROS2 node for vision inspection pipeline."""
    
    def __init__(self):
        super().__init__('vision_inspection_node')
        
        # Declare parameters
        self.declare_parameter('watch_directory', os.path.expanduser('~/seract_orch_ws/scanned_pictures'))
        self.declare_parameter('output_directory', os.path.expanduser('~/seract_orch_ws/scanned_pictures/vision_results'))
        self.declare_parameter('blur_threshold', 120.0)    # kept for param compat, unused
        self.declare_parameter('hand_iou_threshold', 0.02) # kept for param compat, unused
        self.declare_parameter('process_delay_seconds', 0.5)
        self.declare_parameter('batch_process', True)  # Process all 5 views together
        self.declare_parameter('batch_timeout_seconds', 120.0)  # Wait up to 2 minutes for all 5 images
        self.declare_parameter('enable_yolo', False)
        self.declare_parameter('yolo_model_path', '')
        
        # OpenAI parameters (enabled by default)
        self.declare_parameter('enable_openai', True)  # Default: enabled
        self.declare_parameter('openai_api_key', '')  # Empty = use env var
        self.declare_parameter('openai_model', 'gpt-4o-mini')  # Changed to gpt-4o-mini for cost efficiency
        self.declare_parameter('openai_timeout', 30.0)
        self.declare_parameter('openai_max_retries', 3)
        self.declare_parameter('openai_compress_image', True)
        self.declare_parameter('use_heuristic_fallback', True)  # Fallback if OpenAI fails
        
        # Get parameters
        self.watch_directory = Path(self.get_parameter('watch_directory').get_parameter_value().string_value)
        self.output_directory = Path(self.get_parameter('output_directory').get_parameter_value().string_value)
        self.blur_threshold = self.get_parameter('blur_threshold').get_parameter_value().double_value
        self.hand_iou_threshold = self.get_parameter('hand_iou_threshold').get_parameter_value().double_value
        self.process_delay = self.get_parameter('process_delay_seconds').get_parameter_value().double_value
        self.enable_yolo = self.get_parameter('enable_yolo').get_parameter_value().bool_value
        yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        
        # Archive parameters
        self.declare_parameter('archive_enabled', True)  # Enable archiving by default
        self.declare_parameter('archive_directory', os.path.expanduser('~/seract_orch_ws/past_items'))
        self.archive_enabled = self.get_parameter('archive_enabled').get_parameter_value().bool_value
        self.archive_directory = Path(self.get_parameter('archive_directory').get_parameter_value().string_value)
        
        # Get OpenAI parameters
        self.enable_openai = self.get_parameter('enable_openai').get_parameter_value().bool_value
        openai_api_key = self.get_parameter('openai_api_key').get_parameter_value().string_value
        openai_model = self.get_parameter('openai_model').get_parameter_value().string_value
        openai_timeout = self.get_parameter('openai_timeout').get_parameter_value().double_value
        openai_max_retries = self.get_parameter('openai_max_retries').get_parameter_value().integer_value
        openai_compress_image = self.get_parameter('openai_compress_image').get_parameter_value().bool_value
        self.use_heuristic_fallback = self.get_parameter('use_heuristic_fallback').get_parameter_value().bool_value
        
        # Use empty string to indicate "use env var"
        if not openai_api_key:
            openai_api_key = None  # Will read from OPENAI_API_KEY env var
        
        # Note: vision_results folder should already exist - we don't create it at startup
        # It will be created if it doesn't exist when needed, but ideally it should be pre-created
        if not self.output_directory.exists():
            self.get_logger().warn(f"⚠️ vision_results directory does not exist: {self.output_directory}")
            self.get_logger().warn("   Creating it now, but it should exist before starting the node")
            self.output_directory.mkdir(parents=True, exist_ok=True)
        
        # Configure OpenAI (enabled by default, falls back to heuristic if fails)
        if self.enable_openai:
            try:
                from .detection_pipeline import set_openai_config
                set_openai_config(
                    enable=True,
                    api_key=openai_api_key,
                    model=openai_model,
                    timeout=openai_timeout,
                    max_retries=openai_max_retries,
                    compress_image=openai_compress_image
                )
                self.get_logger().info(f"✅ OpenAI Vision API enabled (model: {openai_model})")
                self.get_logger().info("   Will fallback to heuristic/YOLO if OpenAI fails")
            except Exception as e:
                self.get_logger().warn(f"⚠️ OpenAI initialization failed, will use heuristic only: {e}")
                self.enable_openai = False
        else:
            self.get_logger().info("OpenAI disabled (using heuristic/YOLO only)")
        
        # Configure YOLO if enabled
        if self.enable_yolo and yolo_model_path:
            from .detection_pipeline import set_yolo_config
            set_yolo_config(enable=True, model_path=yolo_model_path)
            self.get_logger().info(f"YOLO enabled with model: {yolo_model_path}")
        else:
            self.get_logger().info("YOLO disabled")
        
        # Optional: Publisher for detection results
        self.result_publisher = self.create_publisher(String, '/vision_detection_results', 10)
        self.decision_publisher = self.create_publisher(String, '/vision_decision', 10)
        
        # Batch processing: Group images by item and wait for all 5 views
        self.batch_process = self.get_parameter('batch_process').get_parameter_value().bool_value
        self.batch_timeout = self.get_parameter('batch_timeout_seconds').get_parameter_value().double_value
        self.pending_items = {}  # item_id -> {images: {view: path}, timestamp: time}
        self.batch_lock = threading.Lock()
        
        # Expected views for a complete item
        self.expected_views = {'top', 'front', 'back', 'left', 'right'}
        
        # Time window for grouping images (images within this window are considered same item)
        self.item_grouping_window = 60.0  # seconds - group images captured within 60 seconds
        
        if self.batch_process:
            self.get_logger().info("📦 Batch processing enabled: Will wait for all 5 views before processing")
        
        # Setup file monitoring
        if not WATCHDOG_AVAILABLE:
            self.get_logger().error("watchdog library not available! Install with: pip install watchdog")
            self.get_logger().error("Falling back to polling mode (less efficient)")
            self.setup_polling_mode()
        else:
            self.setup_watchdog_mode()
        
        self.get_logger().info(f"Vision inspection node started (quality gates DISABLED)")
        self.get_logger().info(f"Watching directory: {self.watch_directory}")
        self.get_logger().info(f"Output directory: {self.output_directory}")
        self.get_logger().info(f"Multi-image batch mode: all views sent in ONE OpenAI call")
    
    def setup_watchdog_mode(self):
        """Setup file monitoring using watchdog library."""
        self.event_handler = ImageFileHandler(self)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, str(self.watch_directory), recursive=False)
        self.observer.start()
        self.get_logger().info("File monitoring started (watchdog mode)")
    
    def setup_polling_mode(self):
        """Fallback: Setup polling mode if watchdog not available."""
        self.processed_files = set()
        self.polling_timer = self.create_timer(1.0, self.poll_for_new_images)
        self.get_logger().info("File monitoring started (polling mode)")
    
    def poll_for_new_images(self):
        """Poll directory for new images (fallback if watchdog not available)."""
        if not self.watch_directory.exists():
            return
        
        for ext in ['.png', '.jpg', '.jpeg']:
            for img_path in self.watch_directory.glob(f'*{ext}'):
                if str(img_path) not in self.processed_files:
                    self.processed_files.add(str(img_path))
                    # Wait a bit to ensure file is fully written
                    time.sleep(self.process_delay)
                    self.process_image_async(img_path)
    
    def process_image_async(self, image_path: Path):
        """Process an image asynchronously - either individually or as part of a batch."""
        if self.batch_process:
            # Add to batch and check if ready to process
            self._add_to_batch(image_path)
        else:
            # Process immediately (old behavior)
            thread = threading.Thread(target=self._process_image, args=(image_path,))
            thread.daemon = True
            thread.start()
    
    def _extract_view_name(self, image_path: Path) -> str:
        """
        Extract view name from image filename.
        
        Expected patterns:
        - "{timestamp}_{view}.png" -> view
        - "{view}.png" -> view
        
        Returns:
            view_name or None if cannot parse
        """
        name = image_path.stem  # Without extension
        
        # Try pattern: timestamp_view (e.g., "1771036909374_top")
        parts = name.split('_')
        if len(parts) >= 2:
            # Check if last part is a view name
            view = parts[-1].lower()
            if view in self.expected_views:
                return view
            # Try multi-part view (e.g., "something_top_view")
            view = '_'.join(parts[1:]).lower()
            if view in self.expected_views:
                return view
        
        # Try pattern: just view name (e.g., "top")
        if name.lower() in self.expected_views:
            return name.lower()
        
        # Try pattern: view at end (e.g., "something_top")
        for view in self.expected_views:
            if name.lower().endswith(f'_{view}') or name.lower().endswith(view):
                return view
        
        return None
    
    def _find_or_create_item_id(self, view: str, image_timestamp: float) -> str:
        """
        Find existing item ID for images within time window, or create new one.
        
        Groups images captured within self.item_grouping_window seconds.
        
        Args:
            view: View name
            image_timestamp: Timestamp when image was captured (from filename or file mtime)
        
        Returns:
            item_id string
        """
        with self.batch_lock:
            # Look for existing pending item within time window
            current_time = time.time()
            for item_id, item_data in self.pending_items.items():
                item_time = item_data['timestamp']
                # Check if this image is within the grouping window
                time_diff = abs(image_timestamp - item_time)
                if time_diff <= self.item_grouping_window:
                    # Check if this view is not already in this item
                    if view not in item_data['images']:
                        return item_id
            
            # No matching item found - create new one
            # Use the image timestamp as base for item ID
            new_item_id = f"item_{int(image_timestamp * 1000)}"
            return new_item_id
    
    def _add_to_batch(self, image_path: Path):
        """Add image to batch and process if all 5 views are ready."""
        # Extract view name
        view = self._extract_view_name(image_path)
        
        if not view:
            # Cannot parse view - process individually
            self.get_logger().warn(f"Could not parse view from {image_path.name}, processing individually")
            thread = threading.Thread(target=self._process_image, args=(image_path,))
            thread.daemon = True
            thread.start()
            return
        
        # Get image timestamp (from filename or file modification time)
        image_timestamp = time.time()
        name = image_path.stem
        parts = name.split('_')
        if len(parts) >= 2:
            # Try to extract timestamp from filename
            try:
                timestamp_ms = int(parts[0])
                image_timestamp = timestamp_ms / 1000.0  # Convert ms to seconds
            except ValueError:
                # Use file modification time
                try:
                    image_timestamp = image_path.stat().st_mtime
                except:
                    image_timestamp = time.time()
        else:
            # Use file modification time for images without timestamp in name
            try:
                image_timestamp = image_path.stat().st_mtime
            except:
                image_timestamp = time.time()
        
        # Find or create item ID based on time window
        item_id = self._find_or_create_item_id(view, image_timestamp)
        
        with self.batch_lock:
            # Initialize item if not exists
            if item_id not in self.pending_items:
                self.pending_items[item_id] = {
                    'images': {},
                    'timestamp': image_timestamp  # Use first image's timestamp
                }
                self.get_logger().info(f"📦 Created new batch item: {item_id}")
            
            # Add image to batch
            self.pending_items[item_id]['images'][view] = image_path
            current_count = len(self.pending_items[item_id]['images'])
            self.get_logger().info(f"📸 Added {view} view to batch for item {item_id} ({current_count}/5 views)")
            
            # Check if all 5 views are present
            item_images = self.pending_items[item_id]['images']
            if len(item_images) == 5 and all(v in item_images for v in self.expected_views):
                # All views ready - process batch
                self.get_logger().info(f"✅ All 5 views ready for item {item_id}, processing batch...")
                images_to_process = item_images.copy()
                del self.pending_items[item_id]  # Remove from pending
                
                # Process batch in separate thread
                thread = threading.Thread(target=self._process_batch, args=(item_id, images_to_process))
                thread.daemon = True
                thread.start()
            elif time.time() - self.pending_items[item_id]['timestamp'] > self.batch_timeout:
                # Timeout - process what we have
                self.get_logger().warn(f"⏱️ Batch timeout for item {item_id}, processing {len(item_images)} available views")
                images_to_process = item_images.copy()
                del self.pending_items[item_id]
                
                thread = threading.Thread(target=self._process_batch, args=(item_id, images_to_process))
                thread.daemon = True
                thread.start()
    
    def _process_batch(self, item_id: str, images: dict):
        """
        Process all images for an item in a SINGLE OpenAI API call (no quality gates).

        Args:
            item_id: Unique identifier for the item
            images: Dictionary mapping view names to image paths
        """
        try:
            self.get_logger().info(
                f"🔍 Processing batch for item {item_id} ({len(images)} views) – single API call"
            )

            view_results = {}
            all_defects = []
            overall_decision = "Restock / Resell"
            combined_summary_text = ""
            detection_method_used = None
            object_identification = None

            # Ensure output directory exists
            self.output_directory.mkdir(parents=True, exist_ok=True)

            # ── Try multi-image OpenAI call (all views at once) ──
            if self.enable_openai:
                self.get_logger().info("📡 Sending all views to OpenAI in a single request...")
                (
                    view_defects_map,     # {view: {defects:[...], summary:""}}
                    api_combined_summary,
                    api_decision,
                    success,
                    api_obj_id,
                ) = openai_detection_multi(images)

                if success:
                    detection_method_used = "OpenAI"
                    overall_decision = api_decision or "Restock / Resell"
                    combined_summary_text = api_combined_summary or ""
                    object_identification = api_obj_id

                    for view_name in sorted(images.keys()):
                        image_path = images[view_name]
                        vdata = view_defects_map.get(view_name, {})
                        defects = vdata.get('defects', [])
                        view_summary = vdata.get('summary', '')

                        for d in defects:
                            d['view'] = view_name
                            all_defects.append(d)

                        # Per-view decision follows the overall OpenAI decision
                        # (OpenAI already considered all views together)
                        view_decision = overall_decision

                        view_results[view_name] = {
                            "image_path": str(image_path),
                            "image_name": image_path.stem,
                            "decision": view_decision,
                            "defect_count": len(defects),
                            "defects": defects,
                            "annotated_image_path": None,
                            "openai_metadata": {
                                "summary": view_summary,
                                "overall_decision": overall_decision,
                                "object_identification": object_identification,
                            },
                        }

                    # If OpenAI said "Restock / Resell" but defects were found,
                    # override to "Recycle / Review" for consistency
                    if all_defects and overall_decision == "Restock / Resell":
                        self.get_logger().info(
                            f"⚠️ OpenAI said Restock but {len(all_defects)} defect(s) found – "
                            f"overriding to Recycle / Review"
                        )
                        overall_decision = "Recycle / Review"
                        for vr in view_results.values():
                            if vr.get('defect_count', 0) > 0:
                                vr['decision'] = "Recycle / Review"
                                if 'openai_metadata' in vr:
                                    vr['openai_metadata']['overall_decision'] = "Recycle / Review"
                else:
                    self.get_logger().warn("⚠️ OpenAI multi-image call failed, falling back to per-view heuristic")

            # ── Fallback: per-view heuristic / YOLO (no quality gate) ──
            if not view_results:
                detection_method_used = "Heuristic/YOLO"
                for view_name, image_path in sorted(images.items()):
                    try:
                        detection_results, overlay_path, openai_metadata = run_detectors_on_image(
                            str(image_path),
                            obj_mask=None,
                            save_overlay=True,
                            overlay_path=self.output_directory / f"{item_id}_{view_name}_annotated.png",
                            use_openai=False,
                            use_heuristic_fallback=True,
                        )

                        for d in detection_results:
                            d['view'] = view_name
                            all_defects.append(d)

                        has_defects = len(detection_results) > 0
                        view_decision = "Recycle / Review" if has_defects else "Restock / Resell"
                        if has_defects:
                            overall_decision = "Recycle / Review"

                        view_results[view_name] = {
                            "image_path": str(image_path),
                            "image_name": image_path.stem,
                            "decision": view_decision,
                            "defect_count": len(detection_results),
                            "defects": detection_results,
                            "annotated_image_path": str(overlay_path) if overlay_path else None,
                        }
                    except Exception as e:
                        self.get_logger().error(f"❌ Error processing {view_name} view: {e}")
                        view_results[view_name] = {
                            "image_path": str(image_path),
                            "image_name": image_path.stem,
                            "decision": "Error",
                            "error": str(e),
                        }

            # ── Build combined result JSON ──
            result = {
                "item_id": item_id,
                "timestamp": datetime.now().isoformat(),
                "result": overall_decision,
                "views_processed": list(view_results.keys()),
                "total_views": len(view_results),
                "expected_views": list(self.expected_views),
                "detection_method": detection_method_used,
                "object_identification": object_identification,
                "total_defects": len(all_defects),
                "defects_by_view": {
                    v: len(view_results[v].get('defects', []))
                    for v in view_results
                },
                "all_defects": all_defects,
                "view_results": view_results,
            }

            # Summaries
            if combined_summary_text:
                result["combined_summary"] = combined_summary_text
                result["explanation"] = combined_summary_text
            else:
                if all_defects:
                    defect_types = {}
                    for defect in all_defects:
                        d_type = defect.get('type', 'unknown')
                        defect_types[d_type] = defect_types.get(d_type, 0) + 1
                    summary_parts = [
                        f"Found {len(all_defects)} defect(s) across "
                        f"{len([v for v in view_results.values() if v.get('defect_count', 0) > 0])} view(s):"
                    ]
                    for d_type, count in defect_types.items():
                        summary_parts.append(f"{count} {d_type}(s)")
                    result["explanation"] = " ".join(summary_parts)
                else:
                    result["explanation"] = (
                        "No defects detected across all views. Object appears to be in good condition."
                    )

            # Operator message
            if all_defects:
                views_with_defects = [
                    v for v, r in view_results.items() if r.get('defect_count', 0) > 0
                ]
                result["operator_message"] = (
                    f"⚠️ {len(all_defects)} defect(s) found across "
                    f"{len(views_with_defects)} view(s): "
                    f"{', '.join(views_with_defects)}. {result.get('explanation', '')}"
                )
            else:
                result["operator_message"] = (
                    "✅ No defects detected. Object passed inspection across all views."
                )

            # Save combined JSON
            json_path = self.output_directory / f"{item_id}_result.json"
            with open(json_path, 'w') as f:
                json.dump(result, f, indent=2)

            self.get_logger().info(
                f"✅ Batch processed item {item_id}: {overall_decision} "
                f"({len(all_defects)} total defects across {len(view_results)} views, "
                f"method: {detection_method_used})"
            )

            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_publisher.publish(result_msg)
            
            # Publish decision to vision_decision topic
            decision_msg = String()
            decision_msg.data = overall_decision
            self.decision_publisher.publish(decision_msg)
            self.get_logger().info(f"📡 Published decision to /vision_decision: {overall_decision}")

            # Write decision.json for decision_movement node
            try:
                decision_file = Path(
                    os.path.expanduser(
                        '~/seract_orch_ws/src/so101_motion/so101_motion/decision.json'
                    )
                )
                decision_file.parent.mkdir(parents=True, exist_ok=True)
                with open(decision_file, 'w') as f:
                    json.dump({"decision": overall_decision}, f, indent=2)
                self.get_logger().debug(f"📝 Wrote decision to {decision_file}: {overall_decision}")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to write decision.json: {e}")

            # Archive processed item if enabled
            if self.archive_enabled:
                try:
                    self._archive_processed_item(item_id)
                except Exception as e:
                    self.get_logger().error(f"❌ Error archiving item {item_id}: {e}")
                    import traceback
                    self.get_logger().error(traceback.format_exc())

        except Exception as e:
            self.get_logger().error(f"❌ Error processing batch for item {item_id}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _get_next_item_number(self) -> int:
        """
        Find the next available item number by scanning past_items directory.
        
        Returns:
            Next item number (e.g., 3 if "Item 1" and "Item 2" exist)
        """
        # Ensure archive directory exists
        self.archive_directory.mkdir(parents=True, exist_ok=True)
        
        # Pattern to match "Item X" or "item X" (case-insensitive, with optional spaces)
        pattern = re.compile(r'^item\s*(\d+)$', re.IGNORECASE)
        
        existing_numbers = []
        for item_dir in self.archive_directory.iterdir():
            if not item_dir.is_dir():
                continue
            
            match = pattern.match(item_dir.name)
            if match:
                try:
                    num = int(match.group(1))
                    existing_numbers.append(num)
                except ValueError:
                    continue
        
        if not existing_numbers:
            return 1
        
        return max(existing_numbers) + 1
    
    def _archive_processed_item(self, item_id: str):
        """
        Archive processed item by moving images and vision_results to past_items/Item X/.
        
        Args:
            item_id: The item ID that was just processed
        """
        if not self.archive_enabled:
            return
        
        # Get next item number
        next_item_num = self._get_next_item_number()
        item_folder_name = f"Item {next_item_num}"
        item_folder = self.archive_directory / item_folder_name
        
        self.get_logger().info(f"📦 Archiving processed item to {item_folder_name}...")
        
        # Create item folder structure (both images and vision_results folders)
        images_folder = item_folder / "images"
        results_folder = item_folder / "vision_results"
        images_folder.mkdir(parents=True, exist_ok=True)
        results_folder.mkdir(parents=True, exist_ok=True)
        self.get_logger().debug(f"  Created folders: {images_folder} and {results_folder}")
        
        # Move all images from scanned_pictures/ to past_items/Item X/images/
        moved_images = 0
        for image_file in self.watch_directory.glob("*.png"):
            if image_file.is_file():
                try:
                    dest = images_folder / image_file.name
                    shutil.move(str(image_file), str(dest))
                    moved_images += 1
                    self.get_logger().debug(f"  Moved image: {image_file.name}")
                except Exception as e:
                    self.get_logger().warn(f"  Failed to move {image_file.name}: {e}")
        
        # Also check for .jpg/.jpeg files
        for ext in ['*.jpg', '*.jpeg']:
            for image_file in self.watch_directory.glob(ext):
                if image_file.is_file():
                    try:
                        dest = images_folder / image_file.name
                        shutil.move(str(image_file), str(dest))
                        moved_images += 1
                        self.get_logger().debug(f"  Moved image: {image_file.name}")
                    except Exception as e:
                        self.get_logger().warn(f"  Failed to move {image_file.name}: {e}")
        
        # Move all vision_results from scanned_pictures/vision_results/ to past_items/Item X/vision_results/
        moved_results = 0
        if self.output_directory.exists() and self.output_directory.is_dir():
            for result_file in self.output_directory.iterdir():
                if result_file.is_file():
                    try:
                        dest = results_folder / result_file.name
                        shutil.move(str(result_file), str(dest))
                        moved_results += 1
                        self.get_logger().debug(f"  Moved result: {result_file.name}")
                    except Exception as e:
                        self.get_logger().warn(f"  Failed to move {result_file.name}: {e}")
        
        # Delete vision_results folder after archiving (will be recreated when next image is detected)
        if self.output_directory.exists() and self.output_directory.is_dir():
            try:
                # Check if directory is empty
                if not any(self.output_directory.iterdir()):
                    self.output_directory.rmdir()
                    self.get_logger().debug(f"  Deleted empty vision_results directory")
                else:
                    # Directory not empty - try to remove remaining files and then directory
                    self.get_logger().warn(f"  vision_results directory not empty, attempting cleanup...")
                    for remaining_file in list(self.output_directory.iterdir()):
                        try:
                            if remaining_file.is_file():
                                remaining_file.unlink()
                            elif remaining_file.is_dir():
                                shutil.rmtree(remaining_file)
                        except Exception as e:
                            self.get_logger().warn(f"  Failed to remove {remaining_file.name}: {e}")
                    # Try to remove directory again
                    try:
                        self.output_directory.rmdir()
                        self.get_logger().debug(f"  Deleted vision_results directory after cleanup")
                    except Exception as e:
                        self.get_logger().warn(f"  Could not delete vision_results directory: {e}")
            except Exception as e:
                self.get_logger().warn(f"  Error deleting vision_results directory: {e}")
        
        # Clean up processed_files set - remove entries for archived files
        # This ensures watchdog can detect new files properly
        if hasattr(self, 'event_handler') and hasattr(self.event_handler, 'processed_files'):
            with self.event_handler.processing_lock:
                # Get list of files that currently exist in watch directory
                existing_files = set()
                for ext in ['.png', '.jpg', '.jpeg']:
                    for img_path in self.watch_directory.glob(f'*{ext}'):
                        existing_files.add(str(img_path))
                
                # Remove entries for files that no longer exist (were archived)
                files_to_remove = []
                for tracked_file in list(self.event_handler.processed_files):
                    if tracked_file not in existing_files:
                        files_to_remove.append(tracked_file)
                
                for file_path in files_to_remove:
                    self.event_handler.processed_files.discard(file_path)
                
                if files_to_remove:
                    self.get_logger().debug(f"  Cleaned up {len(files_to_remove)} archived file entries from processed_files set")
                    self.get_logger().debug(f"  processed_files set now has {len(self.event_handler.processed_files)} entries")
        
        self.get_logger().info(
            f"✅ Archived item to {item_folder_name}: "
            f"{moved_images} images, {moved_results} result files"
        )
        self.get_logger().info(f"🔄 Ready for next item - vision_results will be created when first image arrives")
    
    def _process_image(self, image_path: Path):
        """Process a single image: detection only (no quality gate)."""
        try:
            self.get_logger().info(f"Processing image: {image_path.name}")

            # Ensure output directory exists
            self.output_directory.mkdir(parents=True, exist_ok=True)

            # Run detection pipeline (OpenAI by default, falls back to heuristic)
            detection_results, overlay_path, openai_metadata = run_detectors_on_image(
                str(image_path),
                obj_mask=None,
                save_overlay=True,
                overlay_path=self.output_directory / f"{image_path.stem}_annotated.png",
                use_openai=self.enable_openai,
                use_heuristic_fallback=self.use_heuristic_fallback,
            )

            has_defects = len(detection_results) > 0

            if openai_metadata and openai_metadata.get('overall_decision'):
                decision = openai_metadata['overall_decision']
            else:
                decision = "Recycle / Review" if has_defects else "Restock / Resell"

            result = {
                "image_path": str(image_path),
                "image_name": image_path.stem,
                "timestamp": datetime.now().isoformat(),
                "result": decision,
                "defects": detection_results,
                "annotated_image_path": overlay_path if overlay_path else None,
            }

            if openai_metadata:
                result["openai_summary"] = openai_metadata.get('summary', '')
                result["openai_decision"] = openai_metadata.get('overall_decision', '')
                result["object_identification"] = openai_metadata.get('object_identification', None)

                if has_defects:
                    defect_count = len(detection_results)
                    unique_types = list({d.get('type', 'defect') for d in detection_results})
                    result["operator_message"] = (
                        f"⚠️ {defect_count} defect(s) detected: "
                        f"{', '.join(unique_types)}. {openai_metadata.get('summary', '')}"
                    )
                else:
                    result["operator_message"] = "✅ No defects detected. Object appears to be in good condition."
            else:
                if has_defects:
                    unique_types = list({d.get('type', 'defect') for d in detection_results})
                    result["operator_message"] = (
                        f"⚠️ {len(detection_results)} defect(s) detected: {', '.join(unique_types)}"
                    )
                else:
                    result["operator_message"] = "✅ No defects detected."

            json_path = self.output_directory / f"{image_path.stem}_result.json"
            with open(json_path, 'w') as f:
                json.dump(result, f, indent=2)

            detection_method = "OpenAI" if openai_metadata else "Heuristic/YOLO"
            self.get_logger().info(
                f"✅ Processed {image_path.name}: {decision} "
                f"({len(detection_results)} defects, method: {detection_method})"
            )

            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_publisher.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"❌ Error processing {image_path.name}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

            error_result = {
                "image_path": str(image_path),
                "image_name": image_path.stem,
                "timestamp": datetime.now().isoformat(),
                "result": "Error",
                "error": str(e),
            }
            json_path = self.output_directory / f"{image_path.stem}_result.json"
            try:
                with open(json_path, 'w') as f:
                    json.dump(error_result, f, indent=2)
            except Exception:
                pass


def main(args=None):
    """Main entry point for the vision inspection node."""
    rclpy.init(args=args)
    
    node = VisionInspectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down vision inspection node...")
    finally:
        # Stop file observer if using watchdog
        if hasattr(node, 'observer'):
            node.observer.stop()
            node.observer.join()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

