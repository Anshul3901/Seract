#!/bin/bash
# Publish static transform from base_link to camera_link
# Based on measured camera pose: (0.4, 0.0, 0.6) with identity rotation

ros2 run tf2_ros static_transform_publisher \
  0.4 0.0 0.6 \
  0.0 0.0 0.0 1.0 \
  base_link camera_link

