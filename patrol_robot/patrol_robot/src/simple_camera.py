#!/usr/bin/env python3
"""相机驱动 — V4L2+MJPG，启动时自动重试"""
import cv2, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SimpleCamera(Node):
    def __init__(self):
        super().__init__("simple_camera")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/rgb/image_raw", 10)
        self.cap = self._try_open()
        if self.cap is None:
            self.get_logger().error("摄像头打开失败, 节点仍运行(等待设备就绪后需重启)")
        else:
            self.timer = self.create_timer(0.033, self._publish)

    def _try_open(self):
        for i in range(10):
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if cap.isOpened():
                self.get_logger().info(f"摄像头就绪: /dev/video0 640x480 V4L2+MJPG (尝试{i+1}次)")
                return cap
            cap.release()
            self.get_logger().warn(f"摄像头未就绪, 等2秒重试... ({i+1}/10)")
            time.sleep(2)
        return None

    def _publish(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.pub.publish(msg)

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
        print(f"Camera error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
