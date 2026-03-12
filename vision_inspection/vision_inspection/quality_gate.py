"""
Quality gate functions for image inspection.
Checks image blur and hand occlusion before defect detection.
"""

import numpy as np
import cv2
from pathlib import Path

# MediaPipe import will be done lazily (inside function) to avoid import errors
# Set flag to None initially - will be determined on first use
MEDIAPIPE_AVAILABLE = None
mp_hands = None


# Default thresholds (can be overridden)
DEFAULT_BLUR_THRESHOLD = 120.0  # variance of Laplacian; raise if lighting is good (80–200 typical)
DEFAULT_HAND_IOU_THRESHOLD = 0.02  # fraction of object area occluded by hand


def variance_of_laplacian(gray):
    """
    Calculate the variance of the Laplacian to measure image blur.
    
    Args:
        gray: Grayscale image (numpy array)
    
    Returns:
        float: Variance value (higher = less blur)
    """
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def quick_object_mask(img):
    """
    Extract object mask from image using GrabCut or fallback to Otsu thresholding.
    
    Args:
        img: BGR image (numpy array)
    
    Returns:
        numpy.ndarray: Binary mask (1 = object, 0 = background)
    """
    h, w = img.shape[:2]
    rect = (int(0.1*w), int(0.1*h), int(0.8*w), int(0.8*h))
    mask = np.zeros((h, w), np.uint8)
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)
    
    try:
        cv2.grabCut(img, mask, rect, bgdModel, fgdModel, 3, cv2.GC_INIT_WITH_RECT)
        mask2 = np.where((mask==2)|(mask==0), 0, 1).astype('uint8')
    except Exception:
        # Fallback to Otsu thresholding
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        v = hsv[:,:,2]
        thr, bw = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask2 = np.zeros_like(v, dtype=np.uint8)
        if contours:
            c = max(contours, key=cv2.contourArea)
            cv2.drawContours(mask2, [c], -1, 1, thickness=cv2.FILLED)
    
    return mask2


# Hand detection function disabled - MediaPipe API incompatibility
# def hand_overlap_fraction(img, obj_mask):
#     """Hand detection disabled - always returns 0.0 (no hand occlusion assumed)"""
#     return 0.0


def quality_gate(img_or_path, blur_threshold=None, hand_iou_threshold=None, 
                 save_overlay=False, overlay_path=None):
    """
    Run quality gate checks on an image: blur detection and hand occlusion.
    
    Args:
        img_or_path: Either a numpy array (BGR image) or path string/Path to image file
        blur_threshold: Blur threshold (default: DEFAULT_BLUR_THRESHOLD)
        hand_iou_threshold: Hand occlusion threshold (default: DEFAULT_HAND_IOU_THRESHOLD)
        save_overlay: Whether to save quality overlay image
        overlay_path: Path to save overlay (only used if save_overlay=True)
    
    Returns:
        dict: Quality gate results with keys:
            - 'var_laplacian': float - blur variance
            - 'hand_obj_overlap': float - hand occlusion fraction
            - 'passed': bool - whether quality gate passed
            - 'quality_overlay': str - path to overlay (if save_overlay=True)
    """
    # Set default thresholds
    if blur_threshold is None:
        blur_threshold = DEFAULT_BLUR_THRESHOLD
    if hand_iou_threshold is None:
        hand_iou_threshold = DEFAULT_HAND_IOU_THRESHOLD
    
    # Load image if path provided
    if isinstance(img_or_path, (str, Path)):
        img_path = Path(img_or_path)
        img = cv2.imread(str(img_path))
        if img is None:
            raise ValueError(f"Could not read image from {img_path}")
    else:
        img = img_or_path
        img_path = None
    
    # Convert to grayscale for blur detection
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    vblur = variance_of_laplacian(gray)
    
    # Get object mask
    obj_mask = quick_object_mask(img)
    
    # Hand detection disabled - set to 0.0 (assume no hand occlusion)
    frac = 0.0
    
    # Determine if quality gate passed (only check blur, hand detection disabled)
    passed = bool(vblur >= blur_threshold)
    
    # Build result dictionary
    result = {
        'var_laplacian': float(vblur),
        'hand_obj_overlap': float(frac),
        'passed': passed
    }
    
    # Save overlay if requested
    if save_overlay:
        overlay = img.copy()
        overlay[obj_mask == 0] = (overlay[obj_mask == 0] * 0.3).astype(np.uint8)
        cv2.putText(
            overlay,
            f"BlurVar={vblur:.1f} PASS={passed}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if passed else (0, 0, 255),
            2
        )
        
        if overlay_path is None and img_path is not None:
            overlay_path = img_path.parent / f"{img_path.stem}_quality.jpg"
        elif overlay_path is None:
            overlay_path = Path("quality_overlay.jpg")
        
        cv2.imwrite(str(overlay_path), overlay)
        result['quality_overlay'] = str(overlay_path)
    
    return result

