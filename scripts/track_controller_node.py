#!/usr/bin/env python3
"""
Track Controller Node for Lane Following Robot

Subscribes to camera images, performs lane detection, and publishes
velocity commands (geometry_msgs/Twist) to control the robot via micro-ROS.

Topics:
    Subscriptions:
        /camera_image (sensor_msgs/Image): Camera ROI image
    Publications:
        /cmd_vel (geometry_msgs/Twist): Velocity commands for motor controller
        /track_debug_image (sensor_msgs/Image): Debug visualization
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import select


class TrackController(Node):
    def __init__(self):
        super().__init__('track_controller')

        # ---------- PARAMETERS ----------
        self.declare_parameter('image_topic', 'camera_image')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # camera already publishes ROI (Option A)
        self.declare_parameter('row_sample_ratio', 0.60)

        # detection robustness
        self.declare_parameter('min_white_pixels', 45)
        self.declare_parameter('min_lane_width_px', 100)
        self.declare_parameter('max_lane_width_ratio', 0.95)
        self.declare_parameter('max_white_ratio', 0.55)

        # adaptive threshold - tuned for noise reduction
        self.declare_parameter('adapt_block_size', 61)
        self.declare_parameter('adapt_C', -12)

        # morphology - noise removal
        self.declare_parameter('morph_kernel', 7)

        # controller (follow) - tuned for smooth single-line following
        self.declare_parameter('kp', 0.0008)
        self.declare_parameter('base_v', 0.30)
        self.declare_parameter('max_w', 0.50)
        self.declare_parameter('deadband_px', 25.0)

        # ---------- SMOOTHING ----------
        self.declare_parameter('err_alpha', 0.10)

        # ---------- RATE LIMITING ----------
        self.declare_parameter('v_accel_limit', 0.15)
        self.declare_parameter('w_accel_limit', 0.05)

        # ---------- FRAME SKIPPING ----------
        self.declare_parameter('process_every_n_frames', 2)

        # ---------- SINGLE LINE FOLLOWING ----------
        self.declare_parameter('assumed_lane_width_px', 120)

        # ---------- READ PARAMETERS ----------
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.row_sample_ratio = float(self.get_parameter('row_sample_ratio').value)

        self.min_white_pixels = int(self.get_parameter('min_white_pixels').value)
        self.min_lane_width_px = int(self.get_parameter('min_lane_width_px').value)
        self.max_lane_width_ratio = float(self.get_parameter('max_lane_width_ratio').value)
        self.max_white_ratio = float(self.get_parameter('max_white_ratio').value)

        self.adapt_block_size = int(self.get_parameter('adapt_block_size').value)
        self.adapt_C = int(self.get_parameter('adapt_C').value)

        self.morph_kernel = int(self.get_parameter('morph_kernel').value)
        if self.morph_kernel < 1:
            self.morph_kernel = 1
        if self.morph_kernel % 2 == 0:
            self.morph_kernel += 1

        self.kp = float(self.get_parameter('kp').value)
        self.base_v = float(self.get_parameter('base_v').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.deadband_px = float(self.get_parameter('deadband_px').value)

        # smoothing
        self.err_alpha = float(self.get_parameter('err_alpha').value)
        self.err_smooth = 0.0

        # rate limiting
        self.v_accel_limit = float(self.get_parameter('v_accel_limit').value)
        self.w_accel_limit = float(self.get_parameter('w_accel_limit').value)
        self.last_v_sent = 0.0
        self.last_w_sent = 0.0

        # frame skipping
        self.process_every_n = int(self.get_parameter('process_every_n_frames').value)
        self.frame_count = 0

        # single line
        self.assumed_lane_width = int(self.get_parameter('assumed_lane_width_px').value)

        # ---------- MEMORY FOR SINGLE-LINE MODE ----------
        self.last_good_center = None

        # mode/state
        self.mode = "IDLE"  # IDLE / FOLLOW
        self.running = False

        # ---------- LOST HYSTERESIS ----------
        self.lost_count = 0
        self.lost_count_stop = 40
        self.last_v = 0.0
        self.last_w = 0.0

        # ---------- ROS ----------
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.debug_pub = self.create_publisher(Image, 'track_debug_image', 10)

        # Publish cmd_vel instead of serial
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("LANE FOLLOWING ROBOT - MICRO-ROS MODE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Subscribed to /{self.image_topic}")
        self.get_logger().info(f"Publishing cmd_vel to /{self.cmd_vel_topic}")
        self.get_logger().info("Debug image on /track_debug_image")
        self.get_logger().info(f"Frame processing: every {self.process_every_n} frame(s)")
        self.get_logger().info(f"Single line mode: assumed lane width = {self.assumed_lane_width}px")
        self.get_logger().info(f"Lane width validation: {self.min_lane_width_px}px minimum")
        self.get_logger().info(f"Smoothing: err_alpha={self.err_alpha}")
        self.get_logger().info(f"Rate limits: v={self.v_accel_limit}, w={self.w_accel_limit}")
        self.get_logger().info("")
        self.get_logger().info("COMMANDS:")
        self.get_logger().info("  'r' = START following (immediate)")
        self.get_logger().info("  's' = STOP")
        self.get_logger().info("=" * 60)

    def send_cmd_vel(self, v: float, w: float):
        """Publish velocity command as Twist message."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def send_stop(self):
        """Send stop command."""
        self.send_cmd_vel(0.0, 0.0)

    def send_rate_limited(self, v_cmd: float, w_cmd: float):
        """Apply rate limiting and send velocity command."""
        v_diff = v_cmd - self.last_v_sent
        w_diff = w_cmd - self.last_w_sent

        if abs(v_diff) > self.v_accel_limit:
            v_cmd = self.last_v_sent + np.sign(v_diff) * self.v_accel_limit
        if abs(w_diff) > self.w_accel_limit:
            w_cmd = self.last_w_sent + np.sign(w_diff) * self.w_accel_limit

        self.send_cmd_vel(v_cmd, w_cmd)
        self.last_v_sent = v_cmd
        self.last_w_sent = w_cmd

    def keyboard_check(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()
            if cmd == 's':
                self.running = False
                self.mode = "IDLE"
                self.lost_count = 0
                self.last_v = 0.0
                self.last_w = 0.0
                self.err_smooth = 0.0
                self.last_v_sent = 0.0
                self.last_w_sent = 0.0
                self.last_good_center = None
                self.send_stop()
                self.get_logger().warn("STOPPED (manual)")
            elif cmd == 'r':
                self.running = True
                self.mode = "FOLLOW"
                self.lost_count = 0
                self.err_smooth = 0.0
                self.last_good_center = None
                self.get_logger().info("=" * 60)
                self.get_logger().info("START: FOLLOW mode - Robot will move and follow lanes")
                self.get_logger().info("=" * 60)

    def _detect_lane(self, roi_bgr):
        h, w, _ = roi_bgr.shape

        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        block = self.adapt_block_size
        if block % 2 == 0:
            block += 1
        if block < 3:
            block = 3

        bin_img = cv2.adaptiveThreshold(
            blur, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            self.adapt_C
        )

        # Morphology
        k = self.morph_kernel
        kernel = np.ones((k, k), np.uint8)
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel, iterations=1)
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=2)

        dbg = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)

        white_ratio = float(np.mean(bin_img > 0))
        if white_ratio > self.max_white_ratio:
            return None, bin_img, dbg, "too_bright"

        row_y = int(bin_img.shape[0] * self.row_sample_ratio)
        row_y = max(0, min(bin_img.shape[0] - 1, row_y))
        row = bin_img[row_y, :]
        white_idx = np.where(row > 0)[0]

        cv2.line(dbg, (0, row_y), (w - 1, row_y), (255, 0, 0), 2)

        if white_idx.size < self.min_white_pixels:
            return None, bin_img, dbg, "too_few_white"

        gap_px = 5
        gaps = np.where(np.diff(white_idx) > gap_px)[0]
        clusters = np.split(white_idx, gaps + 1)

        if len(clusters) < 1:
            return None, bin_img, dbg, "no_lines"

        img_center = w // 2

        if len(clusters) == 1:
            # SINGLE LINE MODE - Smart line following
            line_x = int(np.mean(clusters[0]))
            lane_width = self.assumed_lane_width

            if line_x < img_center:
                # Seeing LEFT line - robot should be to the right of it
                lane_center = line_x + lane_width // 2
                left_x = line_x
                right_x = line_x + lane_width
                side_txt = "LEFT LINE"
            else:
                # Seeing RIGHT line - robot should be to the left of it
                lane_center = line_x - lane_width // 2
                left_x = line_x - lane_width
                right_x = line_x
                side_txt = "RIGHT LINE"

            # Update last good center for reference
            self.last_good_center = lane_center

            cv2.circle(dbg, (line_x, row_y), 6, (0, 255, 255), -1)
            cv2.putText(dbg, f"SINGLE: {side_txt} - CENTERING", (10, dbg.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            # DUAL LINE MODE
            left_x = int(np.mean(clusters[0]))
            right_x = int(np.mean(clusters[-1]))
            lane_width = right_x - left_x

            lane_center = (left_x + right_x) // 2
            self.last_good_center = lane_center

            cv2.circle(dbg, (left_x, row_y), 6, (0, 255, 0), -1)
            cv2.circle(dbg, (right_x, row_y), 6, (0, 255, 0), -1)

        err = float(lane_center - img_center)
        cv2.circle(dbg, (lane_center, row_y), 7, (0, 0, 255), -1)
        cv2.line(dbg, (img_center, 0), (img_center, dbg.shape[0] - 1), (255, 255, 255), 2)

        return (lane_center, left_x, right_x, err, white_ratio, lane_width), bin_img, dbg, "ok"

    def image_cb(self, msg: Image):
        self.keyboard_check()

        # Frame skipping
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        if not self.running:
            self.send_stop()
            return

        roi = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        det, _, dbg, status = self._detect_lane(roi)

        # ---------- LOST HYSTERESIS ----------
        if status != "ok":
            self.lost_count += 1

            reason_map = {
                "too_bright": "CONTINUING: too bright / no track",
                "too_few_white": "CONTINUING: lane not detected",
                "no_lines": "CONTINUING: no lines detected",
                "bad_width": "CONTINUING: invalid lane width",
            }
            txt = reason_map.get(status, "CONTINUING: lane invalid")

            cv2.putText(dbg, txt, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            cv2.putText(dbg, f"LOST: {self.lost_count}/{self.lost_count_stop} frames", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(dbg, f"MODE: {self.mode}", (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Keep moving with last known values during brief detection losses
            if self.lost_count < self.lost_count_stop:
                v_hold = max(0.0, self.last_v * 0.7)
                w_hold = self.last_w * 0.7
                self.send_rate_limited(v_hold, w_hold)
            else:
                # Continue with reduced speed
                v_hold = max(0.0, self.base_v * 0.5)
                w_hold = self.last_w * 0.5
                self.send_rate_limited(v_hold, w_hold)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
            return

        # Reset lost count
        self.lost_count = 0

        lane_center, left_x, right_x, err, white_ratio, lane_width = det

        # Error smoothing
        self.err_smooth = self.err_alpha * err + (1 - self.err_alpha) * self.err_smooth
        err_use = self.err_smooth

        # ---------- FOLLOW MODE ----------
        self.mode = "FOLLOW"

        if abs(err_use) < self.deadband_px:
            w_cmd = 0.0
            action = "STRAIGHT"
        else:
            w_cmd = self.kp * err_use
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
            action = "RIGHT" if err_use > 0 else "LEFT"

        v_cmd = self.base_v
        self.send_rate_limited(v_cmd, w_cmd)

        self.last_v = v_cmd
        self.last_w = w_cmd

        cv2.putText(dbg, f"MODE: FOLLOW | {action}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.putText(dbg, f"Error: {err:.1f}px | Smooth: {err_use:.1f}px", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(dbg, f"V: {v_cmd:.2f} | W: {w_cmd:.2f}", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(dbg, f"Lane: {lane_width}px | White: {white_ratio:.2f}", (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))

    def destroy_node(self):
        try:
            self.send_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
