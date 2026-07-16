#!/usr/bin/env python3
"""极简相机驱动 — OpenCV 读取 /dev/video0 → ROS2 Image 发布"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SimpleCamera(Node):
    def __init__(self):
        super().__init__("simple_camera")
        self.declare_parameter("device", 0)
        dev = self.get_parameter("device").get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/rgb/image_raw", 10)

        self.cap = cv2.VideoCapture(dev)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开摄像头 /dev/video{dev}")
            raise RuntimeError("Camera open failed")

        self.get_logger().info(f"摄像头就绪: /dev/video{dev} 640x480")
        self.timer = self.create_timer(0.033, self._publish)  # 30 FPS

    def _publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    try:
        rclpy.spin(SimpleCamera())
    except Exception as e:
        print(f"Camera error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
