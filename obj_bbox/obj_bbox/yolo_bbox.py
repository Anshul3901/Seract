import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image as ImageMsg
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np

from ultralytics import YOLO


class YoloBboxNode(Node):
    """
    YOLO-based object detection node.

    Subscribes:
      - image_topic

    Publishes:
      - /object_bbox        (vision_msgs/Detection2DArray)
      - /object_debug_image (sensor_msgs/Image)

    Behavior:
      - ALWAYS publishes /object_bbox every frame
      - If no detection -> publishes detections=[]
      - If detections exist -> publishes the best detection only
    """

    def __init__(self):
        super().__init__('obj_bbox_node')

        self.debug_pub = self.create_publisher(ImageMsg, '/object_debug_image', 10)
        self.pub = self.create_publisher(Detection2DArray, '/object_bbox', 10)

        # Input/model params
        self.declare_parameter('image_topic', '/camera2/color/image_raw')
        self.declare_parameter('model_path', '/home/gixadmin/seract_orch_ws/src/yolo.pt')
        self.declare_parameter('device', 'cpu')   # 'cpu', 'cuda', '0'
        self.declare_parameter('conf_threshold', 0.10)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('max_detections', 10)

        # Optional ROI
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_w', 0)
        self.declare_parameter('roi_h', 0)

        # Optional filtering
        self.declare_parameter('min_bbox_w_px', 8.0)
        self.declare_parameter('min_bbox_h_px', 8.0)
        self.declare_parameter('min_area_ratio', 0.00005)
        self.declare_parameter('max_area_ratio', 0.98)

        self.image_topic = self.get_parameter('image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.max_detections = int(self.get_parameter('max_detections').value)

        self.roi_x = int(self.get_parameter('roi_x').value)
        self.roi_y = int(self.get_parameter('roi_y').value)
        self.roi_w = int(self.get_parameter('roi_w').value)
        self.roi_h = int(self.get_parameter('roi_h').value)

        self.min_bbox_w_px = float(self.get_parameter('min_bbox_w_px').value)
        self.min_bbox_h_px = float(self.get_parameter('min_bbox_h_px').value)
        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)

        self.bridge = CvBridge()

        self.get_logger().info(f'Loading YOLO model from: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(
            f'Subscribing: {self.image_topic} | Publishing: /object_bbox, /object_debug_image'
        )

    def compute_roi_bounds(self, width, height):
        if self.roi_w <= 0 or self.roi_h <= 0:
            return 0, 0, width, height

        x0 = max(0, min(self.roi_x, width - 1))
        y0 = max(0, min(self.roi_y, height - 1))
        x1 = max(x0 + 1, min(x0 + self.roi_w, width))
        y1 = max(y0 + 1, min(y0 + self.roi_h, height))
        return x0, y0, x1, y1

    def publish_no_object(self, header, bgr=None):
        det_arr = Detection2DArray()
        det_arr.header = header
        det_arr.detections = []
        self.pub.publish(det_arr)

        if bgr is not None:
            dbg = bgr.copy()
            cv2.putText(
                dbg, 'NO OBJECT', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            dbg_msg.header = header
            self.debug_pub.publish(dbg_msg)

    def publish_detection(self, header, bgr, bbox, score, class_id='object', label_text=None):
        x0, y0, x1, y1 = bbox
        bw = float(x1 - x0)
        bh = float(y1 - y0)
        cx = float((x0 + x1) / 2.0)
        cy = float((y0 + y1) / 2.0)

        det_arr = Detection2DArray()
        det_arr.header = header

        det = Detection2D()
        det.header = header

        bb = BoundingBox2D()
        bb.center.position.x = cx
        bb.center.position.y = cy
        bb.center.theta = 0.0
        bb.size_x = bw
        bb.size_y = bh
        det.bbox = bb

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)

        det_arr.detections.append(det)
        self.pub.publish(det_arr)

        dbg = bgr.copy()
        cv2.rectangle(dbg, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
        cv2.circle(dbg, (int(cx), int(cy)), 4, (0, 0, 255), -1)

        if label_text is None:
            label_text = f'{class_id}: {score:.2f}'

        cv2.putText(
            dbg,
            label_text,
            (int(x0), max(25, int(y0) - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        dbg_msg.header = header
        self.debug_pub.publish(dbg_msg)

    def choose_best_detection(self, boxes_xyxy, confs, class_ids, names, full_w, full_h):
        best = None
        best_score = -1.0

        for i in range(len(boxes_xyxy)):
            x0, y0, x1, y1 = boxes_xyxy[i]
            conf = float(confs[i])
            cls_id = int(class_ids[i]) if class_ids is not None else -1

            bw = x1 - x0
            bh = y1 - y0
            area_ratio = (bw * bh) / float(max(1, full_w * full_h))

            if bw < self.min_bbox_w_px or bh < self.min_bbox_h_px:
                continue
            if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
                continue

            class_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)

            if conf > best_score:
                best_score = conf
                best = {
                    'bbox': (int(x0), int(y0), int(x1), int(y1)),
                    'score': conf,
                    'class_id': class_name,
                    'label_text': f'{class_name}: {conf:.2f}',
                }

        return best

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        H, W = bgr.shape[:2]
        rx0, ry0, rx1, ry1 = self.compute_roi_bounds(W, H)
        roi = bgr[ry0:ry1, rx0:rx1]

        try:
            results = self.model.predict(
                source=roi,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_detections,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            self.publish_no_object(msg.header, bgr=bgr)
            return

        dbg = bgr.copy()
        cv2.rectangle(dbg, (rx0, ry0), (rx1, ry1), (255, 0, 0), 2)

        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            self.publish_no_object(msg.header, bgr=dbg)
            return

        result = results[0]
        names = result.names if hasattr(result, 'names') else {}

        try:
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy()
        except Exception as e:
            self.get_logger().error(f'Failed to parse YOLO output: {e}')
            self.publish_no_object(msg.header, bgr=dbg)
            return

        # Shift ROI detections back to full-image coordinates
        boxes_xyxy_full = boxes_xyxy.copy()
        boxes_xyxy_full[:, [0, 2]] += rx0
        boxes_xyxy_full[:, [1, 3]] += ry0

        best = self.choose_best_detection(
            boxes_xyxy_full,
            confs,
            class_ids,
            names,
            W,
            H
        )

        if best is None:
            self.publish_no_object(msg.header, bgr=dbg)
            return

        self.publish_detection(
            msg.header,
            dbg,
            best['bbox'],
            best['score'],
            class_id=best['class_id'],
            label_text=best['label_text']
        )


def main():
    rclpy.init()
    node = YoloBboxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()