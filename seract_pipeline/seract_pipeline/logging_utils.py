#!/usr/bin/env python3

import os
import json
import shutil
from datetime import datetime
from pathlib import Path


def log_run(out_dir, data: dict):
    """
    Create a timestamped folder with meta.json and copy /tmp/frame.jpg if present.
    
    Args:
        out_dir: Base output directory where timestamped folders will be created
        data: Dictionary containing metadata to save in meta.json
    
    Returns:
        Path to the created timestamped folder
    """
    # Create timestamped folder name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    timestamp_dir = Path(out_dir) / timestamp
    timestamp_dir.mkdir(parents=True, exist_ok=True)
    
    # Save metadata to meta.json
    meta_path = timestamp_dir / "meta.json"
    with open(meta_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    # Copy /tmp/frame.jpg if present
    frame_source = Path("/tmp/frame.jpg")
    if frame_source.exists():
        frame_dest = timestamp_dir / "frame.jpg"
        shutil.copy2(frame_source, frame_dest)
    
    return timestamp_dir

