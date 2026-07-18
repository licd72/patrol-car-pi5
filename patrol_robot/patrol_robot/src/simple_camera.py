#!/usr/bin/env python3
# 巡逻小车摄像头驱动 — 增强版(带重连+诊断)
import cv2, time, sys, traceback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SimpleCamera(Node):
    def __init__(self):
        super().__init__("simple_camera")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/rgb/image_raw", 10)
        self.cap = None
        self._open_camera()
        if self.cap:
            self.timer = self.create_timer(0.066, self._publish)  # 15fps
            self.get_logger().info("Camera driver ready, 15fps")
        else:
            self.get_logger().error("Camera FAILED after 10 retries!")
            self._retry_timer = self.create_timer(5.0, self._retry_open)

    def _open_camera(self):
        for attempt in range(1, 11):
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            if cap.isOpened():
                # Verify we can actually read
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.cap = cap
                    self.get_logger().info(f"Camera OK: /dev/video0 320x240 MJPG (attempt {attempt})")
                    return
                else:
                    cap.release()
                    self.get_logger().warn(f"Camera opened but read failed (attempt {attempt})")
            else:
                self.get_logger().warn(f"Camera not ready (attempt {attempt}/10)")
            time.sleep(2)
        self.cap = None

    def _retry_open(self):
        if not self.cap:
            self._open_camera()
            if self.cap:
                self._retry_timer.cancel()
                self.timer = self.create_timer(0.066, self._publish)

    def _publish(self):
        if self.cap is None:
            return
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().error("Frame read FAIL, reconnecting...", throttle_duration_sec=3)
                self.cap.release()
                self.cap = None
                self._retry_timer = self.create_timer(2.0, self._retry_open)
                return
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}", throttle_duration_sec=3)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    try:
        node = SimpleCamera()
        rclpy.spin(node)
    except Exception as e:
        print(f"FATAL: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

