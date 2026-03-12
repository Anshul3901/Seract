import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image as ImageMsg
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import numpy as np
import cv2

from ultralytics import SAM


class SamBboxNode(Node):
    """
    Changes vs your original:
    - ALWAYS publishes /object_bbox every frame.
      - If NO object -> publishes Detection2DArray with detections=[]
      - If object -> publishes one Detection2D with bbox + a meaningful score proxy
    - Improves "no object" behavior by:
      - selecting the best mask (not always first)
      - rejecting masks that look like background/table (border-touch, low fill ratio, too small/too big)
    - Debug image:
      - if object -> bbox overlay
      - if no object -> publishes raw image with "NO OBJECT" text
    """

    def __init__(self):
        super().__init__('sam_bbox_node')
        self.debug_pub = self.create_publisher(ImageMsg, '/object_debug_image', 10)

        # Inputs / model
        self.declare_parameter('image_topic', '/camera2/color/image_raw')
        self.declare_parameter('model_name', 'mobile_sam.pt')

        # Basic bbox gates
        self.declare_parameter('min_area_ratio', 0.03)   # raised default (table noise tends to be smaller)
        self.declare_parameter('max_area_ratio', 0.80)
        self.declare_parameter('min_bbox_w_px', 140.0)
        self.declare_parameter('min_bbox_h_px', 120.0)

        # Table/background rejection gates
        self.declare_parameter('border_margin_px', 10)   # reject masks touching image border within margin
        self.declare_parameter('min_fill_ratio', 0.35)   # mask_pixels / bbox_area
        self.declare_parameter('max_fill_ratio', 0.98)   # reject near-solid giant rectangles

        # Mask selection weights
        self.declare_parameter('center_weight', 0.70)    # prefer masks near image center
        self.declare_parameter('area_weight', 0.30)      # but not tiny

        image_topic = self.get_parameter('image_topic').value
        model_name = self.get_parameter('model_name').value

        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)
        self.min_bbox_w_px = float(self.get_parameter('min_bbox_w_px').value)
        self.min_bbox_h_px = float(self.get_parameter('min_bbox_h_px').value)
        self.border_margin_px = int(self.get_parameter('border_margin_px').value)
        self.min_fill_ratio = float(self.get_parameter('min_fill_ratio').value)
        self.max_fill_ratio = float(self.get_parameter('max_fill_ratio').value)
        self.center_weight = float(self.get_parameter('center_weight').value)
        self.area_weight = float(self.get_parameter('area_weight').value)

        self.bridge = CvBridge()
        self.model = SAM(model_name)

        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.pub = self.create_publisher(Detection2DArray, '/object_bbox', 10)

        self.get_logger().info(f"Subscribing: {image_topic} | Publishing: /object_bbox | Model: {model_name}")

    # ---------------- Utility: publish empty detection array ----------------
    def publish_no_object(self, header, bgr=None):
        det_arr = Detection2DArray()
        det_arr.header = header
        det_arr.detections = []
        self.pub.publish(det_arr)

        # Debug image
        if bgr is not None:
            dbg = bgr.copy()
            cv2.putText(dbg, "NO OBJECT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            dbg_msg.header = header
            self.debug_pub.publish(dbg_msg)

    # ---------------- Mask scoring + filtering ----------------
    def mask_to_bbox(self, mask: np.ndarray):
        ys, xs = np.where(mask > 0)
        if xs.size < 50:
            return None
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        bw, bh = (x1 - x0), (y1 - y0)
        if bw <= 2 or bh <= 2:
            return None
        return x0, y0, x1, y1

    def bbox_reject(self, x0, y0, x1, y1, W, H, mask_pixels: int):
        bw = x1 - x0
        bh = y1 - y0

        # size gates
        if bw < self.min_bbox_w_px or bh < self.min_bbox_h_px:
            return True

        # area ratio gate
        area_ratio = (bw * bh) / float(W * H)
        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return True

        # border touch gate (common failure is selecting table)
        m = self.border_margin_px
        if x0 < m or y0 < m or x1 > (W - m) or y1 > (H - m):
            return True

        # fill ratio gate (mask_pixels / bbox_area)
        bbox_area = float(bw * bh)
        fill = float(mask_pixels) / (bbox_area + 1e-6)
        if fill < self.min_fill_ratio or fill > self.max_fill_ratio:
            return True

        return False

    def choose_best_mask(self, masks: np.ndarray, W: int, H: int):
        """
        masks: (N,H,W) float/0-1
        Returns: (best_mask_uint8, best_bbox, score_proxy) or (None,None,0)
        """
        if masks is None or masks.shape[0] == 0:
            return None, None, 0.0

        cx_img, cy_img = W / 2.0, H / 2.0
        diag = (cx_img**2 + cy_img**2) ** 0.5

        best = None
        best_bbox = None
        best_score = -1e9
        best_fill = 0.0

        for i in range(masks.shape[0]):
            m = masks[i]
            mask = (m > 0.5).astype(np.uint8)
            mask_pixels = int(mask.sum())

            bbox = self.mask_to_bbox(mask)
            if bbox is None:
                continue
            x0, y0, x1, y1 = bbox
            bw, bh = (x1 - x0), (y1 - y0)

            if self.bbox_reject(x0, y0, x1, y1, W, H, mask_pixels):
                continue

            # compute centroid (from mask pixels)
            ys, xs = np.where(mask > 0)
            cx = float(xs.mean())
            cy = float(ys.mean())

            center_dist = ((cx - cx_img) ** 2 + (cy - cy_img) ** 2) ** 0.5
            center_term = 1.0 - (center_dist / (diag + 1e-6))  # 1 is best

            area_ratio = (bw * bh) / float(W * H)
            area_term = min(max(area_ratio / 0.25, 0.0), 1.0)  # normalize against 25% view

            score = self.center_weight * center_term + self.area_weight * area_term

            bbox_area = float(bw * bh)
            fill = float(mask_pixels) / (bbox_area + 1e-6)

            # Prefer higher score; tie-breaker on fill (more solid object)
            if score > best_score or (abs(score - best_score) < 1e-6 and fill > best_fill):
                best = mask
                best_bbox = (x0, y0, x1, y1)
                best_score = score
                best_fill = fill

        if best is None:
            return None, None, 0.0

        # score proxy: combine selection score + fill ratio (bounded to [0,1])
        score_proxy = float(np.clip(0.7 * best_score + 0.3 * best_fill, 0.0, 1.0))
        return best, best_bbox, score_proxy

    # ---------------- Main callback ----------------
    def on_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W = bgr.shape[:2]

        # Center point prompt
        cxp, cyp = W // 2, H // 2

        results = self.model.predict(bgr, points=[cxp, cyp], labels=[1], verbose=False)

        # ALWAYS publish something each frame
        if not results or results[0].masks is None or results[0].masks.data is None:
            self.publish_no_object(msg.header, bgr=bgr)
            return

        masks_t = results[0].masks.data  # torch tensor (N,H,W)
        try:
            masks = masks_t.cpu().numpy()
        except Exception:
            self.publish_no_object(msg.header, bgr=bgr)
            return

        best_mask, best_bbox, score_proxy = self.choose_best_mask(masks, W=W, H=H)
        if best_mask is None or best_bbox is None:
            self.publish_no_object(msg.header, bgr=bgr)
            return

        x0, y0, x1, y1 = best_bbox
        bw = float(x1 - x0)
        bh = float(y1 - y0)
        cx = float((x0 + x1) / 2.0)
        cy = float((y0 + y1) / 2.0)

        # Publish detection
        det_arr = Detection2DArray()
        det_arr.header = msg.header

        det = Detection2D()
        det.header = msg.header

        bb = BoundingBox2D()
        bb.center.position.x = cx
        bb.center.position.y = cy
        bb.center.theta = 0.0
        bb.size_x = bw
        bb.size_y = bh
        det.bbox = bb

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "object"
        hyp.hypothesis.score = score_proxy  # now meaningful-ish
        det.results.append(hyp)

        det_arr.detections.append(det)
        self.pub.publish(det_arr)

        # Debug image
        dbg = bgr.copy()
        cv2.rectangle(dbg, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
        cv2.circle(dbg, (int(cx), int(cy)), 4, (0, 0, 255), -1)
        cv2.putText(dbg, f"score={score_proxy:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        dbg_msg.header = msg.header
        self.debug_pub.publish(dbg_msg)


def main():
    rclpy.init()
    node = SamBboxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
