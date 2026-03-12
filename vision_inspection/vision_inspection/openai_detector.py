"""
OpenAI Vision API integration for defect detection.
Uses GPT-4 Vision or GPT-4o to analyze images and detect defects with explanations.
"""

import os
import base64
import json
import time
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Try to import OpenAI
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    OpenAI = None

logger = logging.getLogger(__name__)


class OpenAIDetector:
    """OpenAI Vision API detector for defect detection."""
    
    def __init__(self, 
                 api_key: Optional[str] = None,
                 model: str = "gpt-4o-mini",
                 timeout: float = 30.0,
                 max_retries: int = 3,
                 max_image_size: Tuple[int, int] = (1024, 1024)):
        """
        Initialize OpenAI detector.
        
        Args:
            api_key: OpenAI API key (if None, reads from OPENAI_API_KEY env var)
            model: Model to use ("gpt-4o-mini", "gpt-4o", "gpt-4-vision-preview", etc.)
            timeout: Request timeout in seconds
            max_retries: Maximum number of retry attempts
            max_image_size: Maximum image dimensions (width, height) before resizing
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI package not installed. Install with: pip install openai")
        
        # Get API key from parameter or environment
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("OpenAI API key not provided. Set OPENAI_API_KEY environment variable or pass api_key parameter.")
        
        self.client = OpenAI(api_key=self.api_key)
        self.model = model
        self.timeout = timeout
        self.max_retries = max_retries
        self.max_image_size = max_image_size
        
        logger.info(f"OpenAI detector initialized with model: {model}")
    
    def encode_image_to_base64(self, image_path: Path) -> str:
        """
        Encode image to base64 string for API.
        
        Args:
            image_path: Path to image file
        
        Returns:
            Base64 encoded image string
        """
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
    
    def compress_image(self, image_path: Path, output_path: Optional[Path] = None) -> Path:
        """
        Compress/resize image to reduce API costs.
        
        Args:
            image_path: Path to original image
            output_path: Optional output path (if None, creates temp file)
        
        Returns:
            Path to compressed image
        """
        img = cv2.imread(str(image_path))
        if img is None:
            raise ValueError(f"Could not read image: {image_path}")
        
        h, w = img.shape[:2]
        max_w, max_h = self.max_image_size
        
        # Resize if needed
        if w > max_w or h > max_h:
            scale = min(max_w / w, max_h / h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
            logger.info(f"Resized image from {w}x{h} to {new_w}x{new_h}")
        
        # Save compressed image to /tmp (outside watched directory) to avoid watchdog loop
        if output_path is None:
            import tempfile
            # Use tempfile to create a unique temp file in /tmp
            temp_fd, temp_path = tempfile.mkstemp(suffix='.jpg', prefix='openai_compressed_', dir='/tmp')
            os.close(temp_fd)  # Close file descriptor, we'll use the path
            output_path = Path(temp_path)
        
        # Encode as JPEG with quality 85
        cv2.imwrite(str(output_path), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return output_path
    
    def create_detection_prompt(self, custom_prompt: Optional[str] = None) -> str:
        """
        Create prompt for defect detection.
        
        Args:
            custom_prompt: Optional custom prompt (if None, uses default)
        
        Returns:
            Prompt string
        """
        if custom_prompt:
            return custom_prompt
        
        # Default prompt
        return """You are a quality control inspector analyzing images of manufactured objects.

First, identify what object is being inspected in this image. Then analyze it for defects.

TASK 1: OBJECT IDENTIFICATION
- Identify the type of object

TASK 2: PRIMARY PACKAGING DETECTION
- 

TASK 3: DEFECT DETECTION
Analyze the image and identify any defects such as:
- Scratches (surface abrasions or marks)
- Stains (discoloration, marks, or spots)
- Holes (punctures, openings, or damage)
- Fray edges (damaged or worn edges)

For each defect found, provide:
1. Type of defect (one of: scratch, stain, hole, fray_edge)
2. Bounding box coordinates [x0, y0, x1, y1] in pixels (top-left and bottom-right corners)
3. Confidence score (0.0-1.0)
4. Severity (minor, moderate, or major)
5. Explanation (100 words describing the defect, its location, and appearance)
6. Recommendation (how to address or fix the defect)

TASK 3: FINAL DECISION
Based on your analysis, make a final decision:
- "Restock / Resell": Use this if the object is in good condition with no significant defects, or only minor defects that don't affect functionality. The object is suitable for resale or restocking.
- "Recycle / Review": Use this if the object has significant defects (major defects, multiple defects, or defects that affect functionality), quality issues, or requires further review before being resold.

Return your response as JSON in this exact format:
{
  "object_identification": {
    "object_type": "description of the object type (e.g., 'laptop', 'phone case', 'book')",
  },
  "defects": [
    {
      "type": "scratch",
      "bbox": [x0, y0, x1, y1],
      "confidence": 0.95,
      "severity": "minor",
      "explanation": "A visible scratch approximately 3cm long on the left side of the object, likely caused by surface abrasion during handling.",
      "recommendation": "Surface can be polished to remove minor scratch"
    }
  ],
  "summary": "Overall inspection summary in 2-3 sentences, including object identification and defect assessment",
  "overall_decision": "Restock / Resell" or "Recycle / Review"
}

If no defects are found, return:
{
  "object_identification": {
    "object_type": "description of the object type",
  },
  "defects": [],
  "summary": "No defects detected. Object appears to be in good condition. [Include object identification in summary]",
  "overall_decision": "Restock / Resell"
}

IMPORTANT: Return ONLY valid JSON, no additional text or markdown formatting."""
    
    def parse_openai_response(self, response_text: str) -> Dict:
        """
        Parse OpenAI API response to extract JSON.
        
        Args:
            response_text: Raw response text from API
        
        Returns:
            Parsed dictionary with defects, summary, and decision
        """
        # Try to extract JSON from response
        # Sometimes API returns JSON wrapped in markdown code blocks
        text = response_text.strip()
        
        # Remove markdown code blocks if present
        if text.startswith("```json"):
            text = text[7:]
        elif text.startswith("```"):
            text = text[3:]
        
        if text.endswith("```"):
            text = text[:-3]
        
        text = text.strip()
        
        try:
            result = json.loads(text)
            return result
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse JSON response: {e}")
            logger.error(f"Response text: {text[:500]}")
            # Return error result
            return {
                "object_identification": None,
                "defects": [],
                "summary": "Error parsing API response",
                "overall_decision": "Error",
                "error": str(e)
            }
    
    def detect_defects_with_openai(self, 
                                   image_path: Path,
                                   prompt_template: Optional[str] = None,
                                   compress_image: bool = True) -> Dict:
        """
        Detect defects in image using OpenAI Vision API.
        
        Args:
            image_path: Path to image file
            prompt_template: Optional custom prompt
            compress_image: Whether to compress image before sending (reduces cost)
        
        Returns:
            Dictionary with defects, summary, and decision
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI package not installed")
        
        image_path = Path(image_path)
        if not image_path.exists():
            raise FileNotFoundError(f"Image not found: {image_path}")
        
        # Compress image if requested (save to /tmp to avoid watchdog loop)
        actual_image_path = image_path
        temp_compressed = None
        if compress_image:
            try:
                # Don't pass a path - let compress_image() create temp file in /tmp
                actual_image_path = self.compress_image(image_path, output_path=None)
                temp_compressed = actual_image_path  # Track for cleanup
            except Exception as e:
                logger.warning(f"Image compression failed, using original: {e}")
                actual_image_path = image_path
                temp_compressed = None
        
        try:
            # Encode image
            base64_image = self.encode_image_to_base64(actual_image_path)
            
            # Create prompt
            prompt = self.create_detection_prompt(prompt_template)
            
            # Make API call with retries
            for attempt in range(self.max_retries):
                try:
                    logger.info(f"Calling OpenAI API (attempt {attempt + 1}/{self.max_retries})...")
                    
                    response = self.client.chat.completions.create(
                        model=self.model,
                        messages=[
                            {
                                "role": "user",
                                "content": [
                                    {"type": "text", "text": prompt},
                                    {
                                        "type": "image_url",
                                        "image_url": {
                                            "url": f"data:image/jpeg;base64,{base64_image}"
                                        }
                                    }
                                ]
                            }
                        ],
                        max_tokens=2000,
                        temperature=0.1  # Low temperature for consistent results
                    )
                    
                    # Extract response text
                    response_text = response.choices[0].message.content
                    
                    # Parse response
                    result = self.parse_openai_response(response_text)
                    
                    logger.info(f"OpenAI detection completed: {len(result.get('defects', []))} defects found")
                    return result
                    
                except Exception as e:
                    logger.warning(f"API call attempt {attempt + 1} failed: {e}")
                    if attempt < self.max_retries - 1:
                        # Exponential backoff
                        wait_time = 2 ** attempt
                        logger.info(f"Retrying in {wait_time} seconds...")
                        time.sleep(wait_time)
                    else:
                        # Last attempt failed
                        logger.error(f"All API call attempts failed: {e}")
                        return {
                            "object_identification": None,
                            "defects": [],
                            "summary": f"API call failed after {self.max_retries} attempts: {str(e)}",
                            "overall_decision": "Error",
                            "error": str(e)
                        }
        
        finally:
            # Clean up temporary compressed image
            if temp_compressed and temp_compressed.exists():
                try:
                    temp_compressed.unlink()
                except Exception:
                    pass
    
    # ------------------------------------------------------------------
    # Multi-image (batch) detection – sends ALL views in ONE API call
    # ------------------------------------------------------------------

    def create_multi_image_prompt(self, view_names: List[str]) -> str:
        """Create a prompt that asks OpenAI to analyse multiple views at once."""
        views_str = ", ".join(view_names)
        return f"""You are a quality control inspector analyzing images of a single manufactured object from multiple viewpoints.

You are given {len(view_names)} images of the SAME object from these views (in order): {views_str}.

TASK 1: OBJECT IDENTIFICATION
- Identify the type of object being inspected.

TASK 2: DEFECT DETECTION
For EACH view, look for:
- Scratches (surface abrasions or marks)
- Stains (discoloration, marks, or spots)
- Holes (punctures, openings, or damage)
- Fray edges (damaged or worn edges)

For every defect found, provide:
1. Type (one of: scratch, stain, hole, fray_edge)
2. Bounding box [x0, y0, x1, y1] in pixels
3. Confidence (0.0–1.0)
4. Severity (minor, moderate, or major)
5. Explanation (short description)
6. Recommendation

TASK 3: FINAL DECISION
Based on ALL views combined, make ONE overall decision:
- "Restock / Resell" – object is in good condition, suitable for resale.
- "Recycle / Review" – object has significant defects or needs further review.

Return ONLY valid JSON in this exact format (no markdown, no extra text):
{{
  "object_identification": {{
    "object_type": "e.g. glove, laptop, phone case"
  }},
  "views": {{
    "{view_names[0]}": {{
      "defects": [
        {{
          "type": "scratch",
          "bbox": [x0, y0, x1, y1],
          "confidence": 0.95,
          "severity": "minor",
          "explanation": "...",
          "recommendation": "..."
        }}
      ],
      "summary": "Per-view summary sentence"
    }}
  }},
  "combined_summary": "Overall 2-3 sentence summary across all views",
  "overall_decision": "Restock / Resell"
}}

If a view has no defects, set its "defects" to an empty list.
Make sure EVERY view name ({views_str}) appears as a key inside "views"."""

    def detect_defects_multi_image(self,
                                    image_paths: Dict[str, Path],
                                    compress_image: bool = True) -> Dict:
        """
        Detect defects across multiple images (views) in a SINGLE API call.

        Args:
            image_paths: dict mapping view_name -> image Path
            compress_image: whether to compress before sending

        Returns:
            Parsed dict with per-view results, combined_summary, overall_decision
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI package not installed")

        view_names = list(image_paths.keys())
        prompt = self.create_multi_image_prompt(view_names)

        # Build content list: text prompt + one image_url per view
        content: List[Dict] = [{"type": "text", "text": prompt}]
        temp_files: List[Path] = []

        for view in view_names:
            img_path = Path(image_paths[view])
            if not img_path.exists():
                raise FileNotFoundError(f"Image not found for view {view}: {img_path}")

            actual_path = img_path
            if compress_image:
                try:
                    compressed = self.compress_image(img_path, output_path=None)
                    temp_files.append(compressed)
                    actual_path = compressed
                except Exception:
                    actual_path = img_path

            b64 = self.encode_image_to_base64(actual_path)
            content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{b64}"}
            })

        try:
            for attempt in range(self.max_retries):
                try:
                    logger.info(
                        f"Calling OpenAI multi-image API ({len(view_names)} views, "
                        f"attempt {attempt + 1}/{self.max_retries})..."
                    )
                    response = self.client.chat.completions.create(
                        model=self.model,
                        messages=[{"role": "user", "content": content}],
                        max_tokens=3000,
                        temperature=0.1,
                    )
                    response_text = response.choices[0].message.content
                    result = self.parse_openai_response(response_text)
                    logger.info("OpenAI multi-image detection completed")
                    return result

                except Exception as e:
                    logger.warning(f"Multi-image API attempt {attempt + 1} failed: {e}")
                    if attempt < self.max_retries - 1:
                        wait_time = 2 ** attempt
                        logger.info(f"Retrying in {wait_time}s...")
                        time.sleep(wait_time)
                    else:
                        logger.error(f"All multi-image API attempts failed: {e}")
                        return {
                            "object_identification": None,
                            "views": {},
                            "combined_summary": f"API failed after {self.max_retries} attempts: {e}",
                            "overall_decision": "Error",
                            "error": str(e),
                        }
        finally:
            for tf in temp_files:
                try:
                    if tf.exists():
                        tf.unlink()
                except Exception:
                    pass

    def format_results_for_pipeline(self, openai_result: Dict, image_path: Path) -> List[Dict]:
        """
        Format OpenAI results to match pipeline format.
        
        Args:
            openai_result: Result from detect_defects_with_openai()
            image_path: Path to original image
        
        Returns:
            List of defect dictionaries in pipeline format
        """
        defects = openai_result.get('defects', [])
        formatted = []
        
        for defect in defects:
            formatted.append({
                "type": defect.get('type', 'unknown'),
                "bbox": defect.get('bbox', [0, 0, 0, 0]),
                "confidence": float(defect.get('confidence', 0.0)),
                "source": "openai_vision",
                "explanation": defect.get('explanation', ''),
                "severity": defect.get('severity', 'unknown'),
                "recommendation": defect.get('recommendation', '')
            })
        
        return formatted


def detect_defects_with_openai(image_path: Path,
                               api_key: Optional[str] = None,
                               model: str = "gpt-4o",
                               prompt_template: Optional[str] = None,
                               compress_image: bool = True,
                               timeout: float = 30.0,
                               max_retries: int = 3) -> Dict:
    """
    Convenience function to detect defects using OpenAI Vision API.
    
    Args:
        image_path: Path to image file
        api_key: OpenAI API key (if None, reads from OPENAI_API_KEY env var)
        model: Model to use
        prompt_template: Optional custom prompt
        compress_image: Whether to compress image before sending
        timeout: Request timeout in seconds
        max_retries: Maximum number of retry attempts
    
    Returns:
        Dictionary with defects, summary, and decision
    """
    detector = OpenAIDetector(
        api_key=api_key,
        model=model,
        timeout=timeout,
        max_retries=max_retries
    )
    
    return detector.detect_defects_with_openai(
        image_path=image_path,
        prompt_template=prompt_template,
        compress_image=compress_image
    )

