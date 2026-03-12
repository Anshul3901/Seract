"""
Detection pipeline for defect detection: scratches, stains, holes, fray edges.
Uses heuristic methods with optional YOLO and OpenAI Vision API support.
"""

import math
import numpy as np
import cv2
from pathlib import Path

# Try to import scikit-image
try:
    from skimage import morphology, measure
    SKIMAGE_AVAILABLE = True
except ImportError:
    SKIMAGE_AVAILABLE = False
    morphology = None
    measure = None

# Try to import ultralytics (YOLO) - optional
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    YOLO = None

# Try to import OpenAI detector - optional
try:
    from .openai_detector import OpenAIDetector
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    OpenAIDetector = None

# Damage classes for YOLO
DAMAGE_CLASSES = ["hole", "scratch", "stain", "fray_edge", "hinge_screw_issue"]

# YOLO configuration (disabled by default, as in notebook)
YOLO_ENABLE = False
YOLO_MODEL_NAME = None  # Will be set if YOLO is enabled and model exists

# OpenAI configuration (enabled by default, falls back to heuristic if fails)
OPENAI_ENABLE = False  # Will be set to True when configured
OPENAI_DETECTOR = None  # Will be set if OpenAI is enabled
OPENAI_CONFIG = {
    'api_key': None,
    'model': 'gpt-4o-mini',
    'timeout': 30.0,
    'max_retries': 3,
    'compress_image': True,
    'prompt_template': None
}


def set_yolo_config(enable=False, model_path=None):
    """
    Configure YOLO detection.
    
    Args:
        enable: Whether to enable YOLO detection
        model_path: Path to YOLO model file (.pt)
    """
    global YOLO_ENABLE, YOLO_MODEL_NAME
    YOLO_ENABLE = enable and YOLO_AVAILABLE
    if model_path:
        YOLO_MODEL_NAME = str(Path(model_path))


def set_openai_config(enable=True, api_key=None, model='gpt-4o-mini', timeout=30.0, 
                     max_retries=3, compress_image=True, prompt_template=None):
    """
    Configure OpenAI Vision API detection.
    OpenAI is enabled by default and will fallback to heuristic if it fails.
    
    Args:
        enable: Whether to enable OpenAI detection (default: True)
        api_key: OpenAI API key (if None, reads from OPENAI_API_KEY env var)
        model: Model to use ('gpt-4o', 'gpt-4-vision-preview', etc.)
        timeout: Request timeout in seconds
        max_retries: Maximum number of retry attempts
        compress_image: Whether to compress images before sending (reduces cost)
        prompt_template: Optional custom prompt template
    """
    global OPENAI_ENABLE, OPENAI_DETECTOR, OPENAI_CONFIG
    
    # Try to enable OpenAI if requested and available
    if enable and OPENAI_AVAILABLE:
        OPENAI_CONFIG.update({
            'api_key': api_key,
            'model': model,
            'timeout': timeout,
            'max_retries': max_retries,
            'compress_image': compress_image,
            'prompt_template': prompt_template
        })
        
        try:
            OPENAI_DETECTOR = OpenAIDetector(
                api_key=api_key,
                model=model,
                timeout=timeout,
                max_retries=max_retries
            )
            OPENAI_ENABLE = True
        except Exception as e:
            # If initialization fails (e.g., no API key), disable but don't raise
            # This allows fallback to heuristic
            OPENAI_ENABLE = False
            OPENAI_DETECTOR = None
            import logging
            logger = logging.getLogger(__name__)
            logger.warning(f"OpenAI detector initialization failed, will use heuristic fallback: {e}")
    else:
        OPENAI_ENABLE = False
        OPENAI_DETECTOR = None
        if enable and not OPENAI_AVAILABLE:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning("OpenAI package not available, will use heuristic detection")


def detect_scratches(img, obj_mask):
    """
    Detect scratches: thin elongated edges on object.
    
    Args:
        img: BGR image (numpy array)
        obj_mask: Binary object mask (numpy array)
    
    Returns:
        list: List of tuples (x0, y0, x1, y1, confidence)
    """
    if not SKIMAGE_AVAILABLE:
        return []
    
    try:
        work = cv2.bilateralFilter(img, 5, 50, 50)
        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 150)
        edges[obj_mask == 0] = 0
        
        # Use new scikit-image API (fix deprecated binary_dilation)
        edges_bool = edges > 0
        if hasattr(morphology, 'dilation'):
            # New API
            density = morphology.dilation(edges_bool, footprint=morphology.disk(1)).astype(np.uint8) * 255
        else:
            # Fallback to old API if needed
            density = morphology.binary_dilation(edges_bool, morphology.disk(1)).astype(np.uint8) * 255
        
        num, lab = cv2.connectedComponents(density)
        bboxes = []
        for k in range(1, num):
            ys, xs = np.where(lab == k)
            if len(xs) < 50:
                continue
            x0, x1 = xs.min(), xs.max()
            y0, y1 = ys.min(), ys.max()
            h = y1 - y0 + 1
            w = x1 - x0 + 1
            if max(h, w) / (min(h, w) + 1e-6) < 2.5:
                continue  # elongated
            bboxes.append((x0, y0, x1, y1, float(min(1.0, len(xs) / 500.0))))
        return bboxes
    except Exception:
        return []


def detect_stains(img, obj_mask):
    """
    Detect stains: Lab color outliers inside object region.
    
    Args:
        img: BGR image (numpy array)
        obj_mask: Binary object mask (numpy array)
    
    Returns:
        list: List of tuples (x0, y0, x1, y1, confidence)
    """
    if not SKIMAGE_AVAILABLE:
        return []
    
    try:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
        A, B = lab[:, :, 1], lab[:, :, 2]
        valid = (obj_mask > 0)
        if valid.sum() < 10:
            return []
        
        mA, sA = A[valid].mean(), A[valid].std() + 1e-6
        mB, sB = B[valid].mean(), B[valid].std() + 1e-6
        z = ((A - mA) / sA) ** 2 + ((B - mB) / sB) ** 2
        z[~valid] = 0
        thr = np.percentile(z[valid], 95)
        cand = (z > thr).astype(np.uint8)
        
        # Use new scikit-image API (fix deprecated functions)
        cand_bool = cand.astype(bool)
        if hasattr(morphology, 'remove_small_holes'):
            # Try new API first
            try:
                cand = morphology.remove_small_holes(cand_bool, area_threshold=64).astype(np.uint8)
            except TypeError:
                # New API uses max_size instead of area_threshold
                cand = morphology.remove_small_holes(cand_bool, max_size=64).astype(np.uint8)
        else:
            cand = morphology.remove_small_holes(cand_bool, 64).astype(np.uint8)
        
        if hasattr(morphology, 'remove_small_objects'):
            try:
                cand = morphology.remove_small_objects(cand, min_size=64).astype(np.uint8)
            except TypeError:
                # New API uses max_size
                cand = morphology.remove_small_objects(cand, max_size=64).astype(np.uint8)
        else:
            cand = morphology.remove_small_objects(cand, 64).astype(np.uint8)
        
        # Use new dilation API
        if hasattr(morphology, 'dilation'):
            cand = morphology.dilation(cand, footprint=morphology.disk(3)).astype(np.uint8)
        else:
            cand = morphology.binary_dilation(cand, morphology.disk(3)).astype(np.uint8)
        
        lab_img = measure.label(cand, connectivity=2)
        bboxes = []
        for r in measure.regionprops(lab_img):
            y0, x0, y1, x1 = r.bbox
            area = r.area
            conf = min(1.0, area / 2000.0)
            bboxes.append((x0, y0, x1 - 1, y1 - 1, float(conf)))
        return bboxes
    except Exception:
        return []


def detect_holes(img, obj_mask):
    """
    Detect holes: dark, roughly roundish blobs (snags/holes).
    
    Args:
        img: BGR image (numpy array)
        obj_mask: Binary object mask (numpy array)
    
    Returns:
        list: List of tuples (x0, y0, x1, y1, confidence)
    """
    if not SKIMAGE_AVAILABLE:
        return []
    
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        inv = 255 - gray
        inv[obj_mask == 0] = 0
        thr = cv2.threshold(inv, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        thr = cv2.morphologyEx(thr, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        lab = measure.label(thr > 0, connectivity=2)
        bboxes = []
        for r in measure.regionprops(lab):
            y0, x0, y1, x1 = r.bbox
            area = r.area
            if area < 50:
                continue
            per = r.perimeter + 1e-6
            circ = 4 * math.pi * area / (per * per)
            if circ < 0.2:
                continue
            conf = min(1.0, area / 1500.0)
            bboxes.append((x0, y0, x1 - 1, y1 - 1, float(conf)))
        return bboxes
    except Exception:
        return []


def detect_fray_edge(img, obj_mask):
    """
    Detect fray edges: edge wear near object boundary band.
    
    Args:
        img: BGR image (numpy array)
        obj_mask: Binary object mask (numpy array)
    
    Returns:
        list: List of tuples (x0, y0, x1, y1, confidence)
    """
    if not SKIMAGE_AVAILABLE:
        return []
    
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 120)
        # Use new dilation API
        obj_mask_bool = obj_mask > 0
        if hasattr(morphology, 'dilation'):
            dilated = morphology.dilation(obj_mask_bool, footprint=morphology.disk(5))
        else:
            dilated = morphology.binary_dilation(obj_mask_bool, morphology.disk(5))
        band = dilated ^ obj_mask_bool
        band = band.astype(np.uint8)
        cand = (edges > 0) & (band > 0)
        lab = measure.label(cand, connectivity=2)
        bboxes = []
        for r in measure.regionprops(lab):
            y0, x0, y1, x1 = r.bbox
            area = r.area
            if area < 80:
                continue
            conf = min(1.0, area / 3000.0)
            bboxes.append((x0, y0, x1 - 1, y1 - 1, float(conf)))
        return bboxes
    except Exception:
        return []


def detect_hinge_screw_issue(img, obj_mask):
    """
    Placeholder for hinge/screw issue detection.
    Use YOLO supervised head when you have labels.
    
    Args:
        img: BGR image (numpy array)
        obj_mask: Binary object mask (numpy array)
    
    Returns:
        list: Empty list (placeholder)
    """
    # Placeholder - use YOLO when you have labeled dataset
    return []


def yolo_infer_if_enabled(img):
    """
    Run YOLO inference if enabled and model available.
    
    Args:
        img: BGR image (numpy array)
    
    Returns:
        list: List of tuples (x0, y0, x1, y1, confidence, label)
    """
    if not YOLO_ENABLE or not YOLO_AVAILABLE:
        return []
    
    if YOLO_MODEL_NAME is None or not Path(YOLO_MODEL_NAME).exists():
        return []
    
    try:
        model = YOLO(YOLO_MODEL_NAME)
        res = model.predict(img, verbose=False, conf=0.3, iou=0.5)
        bboxes = []
        for r in res:
            for b in r.boxes:
                x0, y0, x1, y1 = b.xyxy[0].tolist()
                conf = float(b.conf[0].item())
                cls = int(b.cls[0].item())
                label = DAMAGE_CLASSES[cls] if cls < len(DAMAGE_CLASSES) else "damage"
                bboxes.append((int(x0), int(y0), int(x1), int(y1), conf, label))
        return bboxes
    except Exception:
        return []


def openai_detection(img_path, use_heuristic_fallback=True):
    """
    Run OpenAI Vision API detection on an image.
    Falls back to heuristic automatically if OpenAI fails (network issues, API errors, etc.)
    
    Args:
        img_path: Path to image file (must be a file path, not numpy array)
        use_heuristic_fallback: If True, falls back to heuristic if OpenAI fails (default: True)
    
    Returns:
        tuple: (results_list, openai_summary, openai_decision, success_flag, object_identification)
            - results_list: List of defect dicts with OpenAI format (empty if failed)
            - openai_summary: Summary text from OpenAI (None if failed)
            - openai_decision: Overall decision from OpenAI ("Restock / Resell"/"Recycle / Review"/"Error"/None)
            - success_flag: True if OpenAI succeeded, False if should use fallback
            - object_identification: Dict with object_type, description, condition (None if failed)
    """
    if not OPENAI_ENABLE or OPENAI_DETECTOR is None:
        return [], None, None, False, None
    
    img_path = Path(img_path)
    if not img_path.exists():
        return [], None, None, False, None
    
    try:
        # Run OpenAI detection
        openai_result = OPENAI_DETECTOR.detect_defects_with_openai(
            image_path=img_path,
            prompt_template=OPENAI_CONFIG.get('prompt_template'),
            compress_image=OPENAI_CONFIG.get('compress_image', True)
        )
        
        # Check for errors
        if openai_result.get('overall_decision') == 'Error':
            # API call failed - signal to use fallback
            import logging
            logger = logging.getLogger(__name__)
            logger.warning("OpenAI detection returned error, falling back to heuristic")
            return [], None, None, False, None
        
        # Format results
        formatted_results = OPENAI_DETECTOR.format_results_for_pipeline(
            openai_result, img_path
        )
        
        return (
            formatted_results,
            openai_result.get('summary', ''),
            openai_result.get('overall_decision', 'Restock / Resell'),
            True,  # Success
            openai_result.get('object_identification', None)  # Include object identification
        )
        
    except Exception as e:
        # If OpenAI fails (network, API key, etc.), log and signal fallback
        import logging
        logger = logging.getLogger(__name__)
        logger.warning(f"OpenAI detection failed (will use heuristic fallback): {e}")
        
        if use_heuristic_fallback:
            return [], None, None, False, None  # Signal to use fallback
        else:
            return [], f"OpenAI detection failed: {str(e)}", "Error", False, None


def openai_detection_multi(image_paths: dict, use_heuristic_fallback=True):
    """
    Run OpenAI Vision API detection on MULTIPLE images in a single call.

    Args:
        image_paths: dict  view_name -> Path
        use_heuristic_fallback: ignored here (kept for API compat)

    Returns:
        tuple: (view_results_dict, combined_summary, overall_decision, success, object_identification)
            view_results_dict maps view_name -> list of defect dicts
    """
    if not OPENAI_ENABLE or OPENAI_DETECTOR is None:
        return {}, None, None, False, None

    try:
        result = OPENAI_DETECTOR.detect_defects_multi_image(
            image_paths=image_paths,
            compress_image=OPENAI_CONFIG.get('compress_image', True),
        )

        if result.get('overall_decision') == 'Error':
            import logging
            logging.getLogger(__name__).warning("OpenAI multi-image returned error, falling back")
            return {}, None, None, False, None

        # Format per-view defects
        views_raw = result.get('views', {})
        view_defects = {}
        for vname, vdata in views_raw.items():
            formatted = []
            for defect in vdata.get('defects', []):
                formatted.append({
                    "type": defect.get('type', 'unknown'),
                    "bbox": defect.get('bbox', [0, 0, 0, 0]),
                    "confidence": float(defect.get('confidence', 0.0)),
                    "source": "openai_vision",
                    "explanation": defect.get('explanation', ''),
                    "severity": defect.get('severity', 'unknown'),
                    "recommendation": defect.get('recommendation', ''),
                })
            view_defects[vname] = {
                'defects': formatted,
                'summary': vdata.get('summary', ''),
            }

        return (
            view_defects,
            result.get('combined_summary', ''),
            result.get('overall_decision', 'Restock / Resell'),
            True,
            result.get('object_identification', None),
        )

    except Exception as e:
        import logging
        logging.getLogger(__name__).warning(f"OpenAI multi-image failed: {e}")
        return {}, None, None, False, None


def run_detectors_on_image(img_or_path, obj_mask=None, save_overlay=False, overlay_path=None,
                           use_openai=True, use_heuristic_fallback=True):
    """
    Run all detectors on an image and return results.
    
    Args:
        img_or_path: Either a numpy array (BGR image) or path string/Path to image file
        obj_mask: Optional object mask (if None, will be computed)
        save_overlay: Whether to save annotated overlay image
        overlay_path: Path to save overlay (only used if save_overlay=True)
        use_openai: Whether to use OpenAI detection (if enabled)
        use_heuristic_fallback: If True, uses heuristic if OpenAI fails or is disabled
    
    Returns:
        tuple: (results_list, overlay_path_string, openai_metadata)
            - results_list: List of dicts with keys: type, bbox, confidence, source, explanation, severity, recommendation
            - overlay_path_string: Path to saved overlay (or None if not saved)
            - openai_metadata: Dict with 'summary' and 'overall_decision' from OpenAI (or None)
    """
    # Load image if path provided
    if isinstance(img_or_path, (str, Path)):
        img_path = Path(img_or_path)
        img = cv2.imread(str(img_path))
        if img is None:
            raise ValueError(f"Could not read image from {img_path}")
    else:
        img = img_or_path
        img_path = None
    
    results = []
    openai_metadata = None
    openai_succeeded = False
    
    # Try OpenAI detection first (default, falls back to heuristic if fails)
    if use_openai and OPENAI_ENABLE and img_path is not None:
        try:
            openai_results, openai_summary, openai_decision, success, object_identification = openai_detection(
                img_path, use_heuristic_fallback=use_heuristic_fallback
            )
            
            if success and openai_results is not None:  # OpenAI succeeded
                results.extend(openai_results)
                openai_metadata = {
                    'summary': openai_summary,
                    'overall_decision': openai_decision,
                    'object_identification': object_identification
                }
                openai_succeeded = True
        except Exception as e:
            # OpenAI error - will fall back to heuristic
            import logging
            logger = logging.getLogger(__name__)
            logger.warning(f"OpenAI detection exception, using heuristic fallback: {e}")
    
    # Fallback to heuristic/YOLO detectors if:
    # 1. OpenAI not enabled/available, OR
    # 2. OpenAI failed (network issues, API key problems, etc.), OR
    # 3. No image path (can't use OpenAI with numpy array), OR
    # 4. OpenAI returned no results
    if not openai_succeeded:
        # Get object mask if not provided
        if obj_mask is None:
            from .quality_gate import quick_object_mask
            obj_mask = quick_object_mask(img)
        
        # Run heuristic detectors
        detectors = [
            ("scratch", detect_scratches),
            ("stain", detect_stains),
            ("hole", detect_holes),
            ("fray_edge", detect_fray_edge),
            ("hinge_screw_issue", detect_hinge_screw_issue),
        ]
        
        for label, func in detectors:
            try:
                boxes = func(img, obj_mask)
                for (x0, y0, x1, y1, conf) in boxes:
                    results.append({
                        "type": label,
                        "bbox": [int(x0), int(y0), int(x1), int(y1)],
                        "confidence": float(conf),
                        "source": "heuristic"
                    })
            except Exception:
                continue  # Skip detector if it fails
        
        # Optional YOLO (only if not using OpenAI or OpenAI failed)
        try:
            yolo_boxes = yolo_infer_if_enabled(img)
            for (x0, y0, x1, y1, conf, label) in yolo_boxes:
                results.append({
                    "type": label,
                    "bbox": [int(x0), int(y0), int(x1), int(y1)],
                    "confidence": float(conf),
                    "source": "yolo"
                })
        except Exception:
            pass  # Skip YOLO if it fails
    
    # Create overlay if requested
    overlay_path_str = None
    if save_overlay:
        overlay = img.copy()
        cmap = {
            "hole": (255, 0, 0),
            "scratch": (0, 255, 0),
            "stain": (0, 0, 255),
            "fray_edge": (255, 255, 0),
            "hinge_screw_issue": (255, 0, 255)
        }
        
        for r in results:
            x0, y0, x1, y1 = r["bbox"]
            color = cmap.get(r["type"], (255, 255, 255))
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, 2)
            
            # Show more info for OpenAI results
            label_text = f"{r['type']}:{r['confidence']:.2f}"
            if r.get('source') == 'openai_vision' and r.get('severity'):
                label_text += f" [{r['severity']}]"
            
            cv2.putText(
                overlay,
                label_text,
                (x0, max(0, y0 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1
            )
        
        if overlay_path is None and img_path is not None:
            overlay_path = img_path.parent / f"{img_path.stem}_annotated.png"
        elif overlay_path is None:
            overlay_path = Path("detection_overlay.png")
        
        cv2.imwrite(str(overlay_path), overlay)
        overlay_path_str = str(overlay_path)
    
    return results, overlay_path_str, openai_metadata

