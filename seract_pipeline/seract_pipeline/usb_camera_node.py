#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import subprocess
import os


class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Declare parameters
        # Try video2 first (sometimes video1 is metadata, video2 is actual video)
        self.declare_parameter('video_device', '/dev/video2')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30.0)
        
        # Get parameters
        self.video_device_param = self.get_parameter('video_device').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.camera_enabled = True
        self.cap = None
        self._open_camera_or_raise()
        
        # Create publisher - publish to camera2 namespace for wrist camera
        self.declare_parameter('image_topic', '/camera2/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        
        # Create camera info publisher (optional, but useful)
        from sensor_msgs.msg import CameraInfo
        camera_info_topic = image_topic.replace('/image_raw', '/camera_info')
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Store frame_id
        self.frame_id = frame_id
        
        # Create timer for publishing images
        timer_period = 1.0 / fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.set_enabled_srv = self.create_service(SetBool, '/usb_camera_node/set_enabled', self.set_enabled_cb)
        
        self.get_logger().info('USB Camera node started')

    def _release_camera(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.cap = None

    def _open_camera_or_raise(self):
        self._release_camera()

        device = self.video_device_param          # e.g. /dev/video_opencv_side
        real_path = os.path.realpath(device)       # resolve symlink → /dev/video6
        self.get_logger().info(f'Opening camera: {device} (resolved: {real_path})')

        if not os.path.exists(real_path):
            raise RuntimeError(f'Camera device does not exist: {device} -> {real_path}')

        # Set MJPG format via v4l2-ctl before opening
        try:
            subprocess.run(
                ['v4l2-ctl', '--device', real_path,
                 f'--set-fmt-video=width={self.image_width},height={self.image_height},pixelformat=MJPG'],
                capture_output=True, timeout=2, check=False
            )
        except Exception:
            pass

        # Open by real device path (NOT numeric index — indices don't match device numbers)
        cap = cv2.VideoCapture(real_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'cv2.VideoCapture failed to open {real_path}')

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Verify we can actually read a frame
        ok = False
        for _ in range(15):
            ret, frame = cap.read()
            if ret and frame is not None:
                ok = True
                break
            import time
            time.sleep(0.1)

        if not ok:
            cap.release()
            raise RuntimeError(f'Camera opened but no frames received from {real_path}')

        self.cap = cap
        self.get_logger().info(
            f'✅ Camera opened successfully: {real_path} '
            f'({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})'
        )

    def set_enabled_cb(self, req, resp):
        try:
            enable = bool(req.data)
            if enable == self.camera_enabled:
                resp.success = True
                resp.message = f'Camera already {"enabled" if enable else "disabled"}'
                return resp

            if not enable:
                self.camera_enabled = False
                self._release_camera()
                self.get_logger().warn('USB camera disabled and released for external policy use.')
            else:
                self._open_camera_or_raise()
                self.camera_enabled = True
                self.get_logger().info('USB camera re-enabled and reopened.')

            resp.success = True
            resp.message = 'ok'
            return resp
        except Exception as e:
            resp.success = False
            resp.message = str(e)
            return resp
    
    def timer_callback(self):
        """Capture and publish image."""
        if not self.camera_enabled:
            return
        # Check if camera is still opened
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('Camera is not opened')
            return
        
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            # Only warn occasionally to avoid spam
            if not hasattr(self, '_frame_fail_count'):
                self._frame_fail_count = 0
            self._frame_fail_count += 1
            if self._frame_fail_count % 30 == 1:  # Warn every 30th failure
                self.get_logger().warn(f'Failed to capture frame (total failures: {self._frame_fail_count})')
            return
        
        # Reset failure count on success
        if hasattr(self, '_frame_fail_count'):
            if self._frame_fail_count > 0:
                self.get_logger().info(f'Camera frame capture recovered after {self._frame_fail_count} failures')
            self._frame_fail_count = 0
        
        try:
            # Convert BGR to RGB (OpenCV uses BGR, ROS uses RGB)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.frame_id
            
            # Publish image
            self.publisher_.publish(img_msg)
            
            # Publish camera info (basic)
            from sensor_msgs.msg import CameraInfo
            camera_info = CameraInfo()
            camera_info.header.stamp = img_msg.header.stamp
            camera_info.header.frame_id = self.frame_id
            camera_info.width = frame.shape[1]
            camera_info.height = frame.shape[0]
            # Note: distortion parameters and camera matrix would need calibration
            self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
    
    def destroy_node(self):
        """Clean up resources."""
        self._release_camera()
        self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        usb_camera_node = USBCameraNode()
        rclpy.spin(usb_camera_node)
    except KeyboardInterrupt:
        if 'usb_camera_node' in locals():
            usb_camera_node.get_logger().info("Shutting down...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            if 'usb_camera_node' in locals():
                usb_camera_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors (context may already be shut down)


if __name__ == '__main__':
    main()

