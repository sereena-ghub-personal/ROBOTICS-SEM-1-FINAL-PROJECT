#!/usr/bin/env python3
"""
Camera Recorder Node for Lane Following Robot

Records camera and debug streams to MP4 files for debugging
and analysis.

Topics:
    Subscriptions:
        /camera_image (sensor_msgs/Image): Raw camera ROI
        /track_debug_image (sensor_msgs/Image): Debug visualization
"""
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraROSRecorder(Node):
    def __init__(self):
        super().__init__('camera_ros_recorder')

        self.bridge = CvBridge()

        # ---- topics to record ----
        self.raw_topic = 'camera_image'
        self.dbg_topic = 'track_debug_image'

        self.sub_raw = self.create_subscription(Image, self.raw_topic, self.raw_cb, 10)
        self.sub_dbg = self.create_subscription(Image, self.dbg_topic, self.dbg_cb, 10)

        # ---- output dir ----
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = os.path.expanduser("~/ros2_ws/src/final_project/recordings")
        os.makedirs(base_dir, exist_ok=True)

        self.raw_path = os.path.join(base_dir, f"camera_raw_{ts}.mp4")
        self.dbg_path = os.path.join(base_dir, f"camera_debug_{ts}.mp4")

        # ---- writers ----
        self.raw_writer = None
        self.dbg_writer = None
        self.fps = 20.0

        self.raw_frames = 0
        self.dbg_frames = 0

        self.get_logger().info(f"Recording RAW  /{self.raw_topic}  -> {self.raw_path}")
        self.get_logger().info(f"Recording DEBUG/{self.dbg_topic}  -> {self.dbg_path}")
        self.get_logger().info("Stop with Ctrl+C")

    def _to_bgr(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def _ensure_writer(self, path, frame):
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        return cv2.VideoWriter(path, fourcc, self.fps, (w, h))

    def _overlay_stamp(self, frame, label, idx):
        ts_text = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        cv2.putText(frame, f"{label}  frame={idx}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, ts_text, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def raw_cb(self, msg: Image):
        frame = self._to_bgr(msg)

        if self.raw_writer is None:
            self.raw_writer = self._ensure_writer(self.raw_path, frame)
            self.get_logger().info(f"RAW writer started ({frame.shape[1]}x{frame.shape[0]} @ {self.fps}fps)")

        self.raw_frames += 1
        out = frame.copy()
        self._overlay_stamp(out, "RAW", self.raw_frames)
        self.raw_writer.write(out)

    def dbg_cb(self, msg: Image):
        frame = self._to_bgr(msg)

        if self.dbg_writer is None:
            self.dbg_writer = self._ensure_writer(self.dbg_path, frame)
            self.get_logger().info(f"DEBUG writer started ({frame.shape[1]}x{frame.shape[0]} @ {self.fps}fps)")

        self.dbg_frames += 1
        out = frame.copy()
        self._overlay_stamp(out, "DEBUG", self.dbg_frames)
        self.dbg_writer.write(out)

    def destroy_node(self):
        if self.raw_writer is not None:
            self.raw_writer.release()
        if self.dbg_writer is not None:
            self.dbg_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraROSRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping recorder...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
