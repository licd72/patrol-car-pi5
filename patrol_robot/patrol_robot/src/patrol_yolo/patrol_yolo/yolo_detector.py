#!/usr/bin/env python3
"""
巡逻小车 YOLO 异常检测节点 (RPi5 优化版)

基于 YOLOv5n + ONNX Runtime, 针对树莓派5 ARM64 优化:
- ONNX Runtime 比 PyTorch 快 2-3 倍
- 可配置检测间隔, 降低 CPU 占用 (默认 0.5s/次)
- 仅检测 person/car/motorcycle 三类异常目标
- 支持 USB 摄像头和 Astra 深度相机

ROS2 接口:
  Subscribers:
    /camera/rgb/image_raw  (sensor_msgs/Image)
  Publishers:
    /patrol/detections     (vision_msgs/Detection2DArray)
    /patrol/alert_image    (sensor_msgs/Image) — 检测到异常时的抓拍

参数 (通过 yolo_params.yaml 配置):
  model_path:       ONNX 模型路径
  confidence:       置信度阈值 (默认 0.5)
  detect_interval:  检测间隔秒数 (默认 0.5, 即 2 FPS)
  target_classes:   关注的类别 ID (默认 [0,2,3] = person,car,motorcycle)
  publish_debug:    是否发布调试图像 (默认 False)
"""

import time
import threading
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge

# ── COCO 类别 (仅关注 person / car / motorcycle) ──
COCO_CLASSES = {
    0: "person",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
}


class YoloDetector(Node):
    """YOLO 异常目标检测 ROS2 节点"""

    def __init__(self):
        super().__init__("patrol_yolo")

        # ── 声明参数 ──
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("detect_interval", 0.5)
        self.declare_parameter("target_classes", [0, 2, 3])
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 640)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.confidence = self.get_parameter("confidence").get_parameter_value().double_value
        self.detect_interval = self.get_parameter("detect_interval").get_parameter_value().double_value
        self.target_classes = self.get_parameter("target_classes").get_parameter_value().integer_array_value
        self.publish_debug = self.get_parameter("publish_debug").get_parameter_value().bool_value
        self.input_width = self.get_parameter("input_width").get_parameter_value().integer_value
        self.input_height = self.get_parameter("input_height").get_parameter_value().integer_value

        # ── 加载 ONNX 模型 ──
        if not model_path:
            # 默认路径: 包同目录下的 models/
            model_path = str(Path(__file__).parent / "models" / "yolov5n.onnx")

        self.get_logger().info(f"加载 ONNX 模型: {model_path}")
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],  # RPi5 无 CUDA
        )
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"模型加载完成, 输入层: {self.input_name}")

        # ── CV Bridge ──
        self.bridge = CvBridge()

        # ── 状态变量 ──
        self._lock = threading.Lock()
        self._last_detect_time = 0.0
        self._frame_count = 0
        self._detect_count = 0

        # ── 订阅 & 发布 ──
        self.sub = self.create_subscription(
            Image, "/camera/rgb/image_raw", self.image_callback, 10
        )
        self.pub_detections = self.create_publisher(
            Detection2DArray, "/patrol/detections", 10
        )
        self.pub_alert_img = self.create_publisher(
            Image, "/patrol/alert_image", 10
        )

        self.get_logger().info(
            f"patrol_yolo 启动 | "
            f"目标类别: {[COCO_CLASSES.get(c, str(c)) for c in self.target_classes]} | "
            f"检测间隔: {self.detect_interval}s | "
            f"置信度阈值: {self.confidence}"
        )

    # ── 图像回调 (帧率控制, 不丢帧) ──
    def image_callback(self, msg: Image):
        """接收相机图像, 按间隔执行检测"""
        now = time.time()

        # 跳帧: 距上次检测不足 interval 秒则跳过
        if now - self._last_detect_time < self.detect_interval:
            return

        self._last_detect_time = now
        self._frame_count += 1

        try:
            # ROS 图像 → OpenCV BGR
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 执行检测
            detections = self._detect(cv_img)

            # 发布结果（仅当有实际检测）
            if detections.detections:
                self.pub_detections.publish(detections)
                self._detect_count += 1

                # 异常目标出现时发布抓拍图
                alert_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                alert_img.header = msg.header
                self.pub_alert_img.publish(alert_img)

            # 定期打印统计
            if self._frame_count % 20 == 0:
                self.get_logger().info(
                    f"统计: {self._frame_count} 帧检测, "
                    f"{self._detect_count} 次命中 "
                    f"({self._detect_count / max(self._frame_count, 1) * 100:.1f}%)"
                )

        except Exception as e:
            self.get_logger().error(f"检测异常: {e}", throttle_duration_sec=5)

    # ── 核心: YOLO ONNX 推理 ──
    def _detect(self, img: np.ndarray) -> Detection2DArray:
        """
        执行 ONNX 推理, 返回 Detection2DArray

        Args:
            img: BGR 图像 (H, W, 3)

        Returns:
            Detection2DArray 消息 (可能为空)
        """
        h, w = img.shape[:2]

        # 1. 预处理: resize + normalize
        blob = cv2.resize(img, (self.input_width, self.input_height))
        blob = blob.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))  # HWC → CHW
        blob = np.expand_dims(blob, axis=0)    # → (1, 3, 640, 640)

        # 2. ONNX 推理
        with self._lock:
            outputs = self.session.run(None, {self.input_name: blob})

        # YOLOv5 输出: (1, 25200, 85) — 85 = 4(bbox) + 1(obj) + 80(cls)
        predictions = outputs[0][0]  # (25200, 85)

        # 3. 后处理: NMS + 过滤
        boxes, scores, class_ids = self._postprocess(predictions, w, h)

        # 4. 构造 ROS2 消息
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"

        for box, score, cls_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box
            det = Detection2D()

            # 边界框
            bbox = BoundingBox2D()
            bbox.center.x = float((x1 + x2) / 2.0)
            bbox.center.y = float((y1 + y2) / 2.0)
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            det.bbox = bbox

            # 类别 + 置信度 (ROS2 Foxy: 扁平结构)
            hyp = ObjectHypothesisWithPose()
            hyp.id = str(cls_id)
            hyp.score = float(score)
            det.results.append(hyp)

            msg.detections.append(det)

        return msg

    # ── 后处理: NMS ──
    def _postprocess(
        self, predictions: np.ndarray, img_w: int, img_h: int
    ) -> tuple:
        """
        YOLO 输出后处理: 置信度过滤 → 坐标缩放 → NMS

        Returns:
            (boxes, scores, class_ids)
        """
        # 解包: 兼容多种 YOLO 输出格式
        # YOLOv5n:  (25200, 85)  → boxes(4), obj(1), cls(80)
        # YOLOv5nu: (84, 8400)    → 需要 transpose → (8400, 84)
        if predictions.ndim == 2 and predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T  # (84, 8400) → (8400, 84)

        if predictions.shape[1] == 85:  # 经典 YOLOv5
            raw_boxes = predictions[:, :4]
            obj_conf = predictions[:, 4:5]
            cls_probs = predictions[:, 5:]
            scores = obj_conf * cls_probs
        elif predictions.shape[1] == 84:  # YOLOv5nu
            raw_boxes = predictions[:, :4]
            cls_probs = predictions[:, 4:]
            scores = cls_probs  # 无 objectness, 直接 class scores
        else:
            self.get_logger().error(f"未知预测格式: {predictions.shape}")
            return [], [], []
        class_ids = np.argmax(scores, axis=1)  # (25200,)
        max_scores = np.max(scores, axis=1)    # (25200,)

        # 筛选: 置信度 + 目标类别
        mask = (max_scores >= self.confidence) & np.isin(class_ids, self.target_classes)
        if not mask.any():
            return [], [], []

        raw_boxes = raw_boxes[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]

        # cxcywh → xyxy (仍归一化)
        boxes_xyxy = self._cxcywh_to_xyxy(raw_boxes)

        # 缩放到原图尺寸
        boxes_xyxy[:, [0, 2]] *= img_w
        boxes_xyxy[:, [1, 3]] *= img_h
        boxes_xyxy = boxes_xyxy.astype(int)

        # NMS
        keep = self._nms(boxes_xyxy, max_scores, iou_threshold=0.45)

        return boxes_xyxy[keep], max_scores[keep], class_ids[keep]

    @staticmethod
    def _cxcywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
        """(cx, cy, w, h) → (x1, y1, x2, y2)"""
        result = np.zeros_like(boxes)
        result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # x1
        result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # y1
        result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # x2
        result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # y2
        return result

    @staticmethod
    def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list:
        """纯 NumPy NMS 实现"""
        x1 = boxes[:, 0].astype(float)
        y1 = boxes[:, 1].astype(float)
        x2 = boxes[:, 2].astype(float)
        y2 = boxes[:, 3].astype(float)
        areas = (x2 - x1) * (y2 - y1)

        order = scores.argsort()[::-1]
        keep = []

        while len(order) > 0:
            i = order[0]
            keep.append(int(i))

            if len(order) == 1:
                break

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter)

            remain = np.where(iou <= iou_threshold)[0]
            order = order[remain + 1]

        return keep


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()

    # 使用 MultiThreadedExecutor, 避免图像回调阻塞
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
