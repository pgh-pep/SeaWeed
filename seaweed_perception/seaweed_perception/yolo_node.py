#!/usr/bin/env python3

import os
import time

import torch

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# import cv2
import numpy as np
from numpy.typing import NDArray

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from seaweed_interfaces.msg import BoundingBox, Detection

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.utils.plotting import Annotator


# TODO: TensorRT if on jetson (need to generate tensor engine,
# select that as model instead of .pt if on jetson, easy way to detect if on jetson
# is if we are using ):
# https://docs.ultralytics.com/guides/nvidia-jetson/#install-onnxruntime-gpu_2
class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("image_topic", "/wamv/sensors/cameras/camera_sensor/optical/image_raw")
        self.declare_parameter("model", "sim_buoy_matrix.pt")
        self.declare_parameter("use_cuda", False)
        self.declare_parameter("debug_w_visualizer", True)

        perception_prefix = get_package_share_directory("seaweed_perception")

        self.image_topic = str(self.get_parameter("image_topic").value)
        model = str(self.get_parameter("model").value)
        self.debug_w_visualizer = bool(self.get_parameter("debug_w_visualizer").value)
        self.use_cuda = bool(self.get_parameter("use_cuda").value)

        self.model_path = os.path.join(perception_prefix, "models", model)

        self.YOLO = YOLO(self.model_path)

        self.cv_bridge = CvBridge()

        self.camera_frame = "wamv/base_link/camera_sensor_optical"

        # NOTE: will add standardized QOS config file in utils
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )

        self.yolo_timer = self.create_timer(0.05, self.yolo_callback)  # 20 hz
        self.confidence = 0.3

        self.latest_image: NDArray[np.uint8] | None = None
        self.height: int = 0
        self.width: int = 0

        self.device = "cuda" if self.use_cuda and torch.cuda.is_available() else "cpu"

        self.detection_pub = self.create_publisher(Detection, "/cv_detections", 10)
        self.debug_yolo_pub = self.create_publisher(Image, "/debug/yolo", image_qos)

        self.get_logger().info(f"YOLO node w/ {model} on {self.device}")

    def image_callback(self, msg: Image) -> None:
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.height, self.width = self.latest_image.shape[:2]
        except CvBridgeError as e:
            self.get_logger().error(f"image callback error: {str(e)}")

    def yolo_callback(self) -> None:
        if self.latest_image is None:
            return

        start_time: float = time.time()

        predictions: list[Results] = self.YOLO.predict(
            source=self.latest_image,
            verbose=False,
            stream=False,
            conf=self.confidence,
            device=self.device,
        )

        inference_time: float = time.time() - start_time

        results: Results = predictions[0]

        detection_msg = Detection()

        detection_msg.header = Header()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = self.camera_frame

        detection_msg.image_width = self.width
        detection_msg.image_height = self.height
        detection_msg.inference_time = inference_time

        detection_msg.detections = []

        if results.boxes is not None and len(results.boxes) > 0:
            for box in results.boxes:
                bbox = BoundingBox()

                # box coordinates in xyxy format
                x1: float
                y1: float
                x2: float
                y2: float

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # top-left corner and width/height
                bbox.x = float(x1)
                bbox.y = float(y1)
                bbox.width = float(x2 - x1)
                bbox.height = float(y2 - y1)

                bbox.x_normalized = float((x1 + x2) / 2 / self.width)
                bbox.y_normalized = float((y1 + y2) / 2 / self.height)
                bbox.width_normalized = float((x2 - x1) / self.width)
                bbox.height_normalized = float((y2 - y1) / self.height)

                bbox.class_id = int(box.cls.item())
                bbox.label = results.names[bbox.class_id]
                bbox.confidence = float(box.conf.item())

                # TO LOG LABELS & CONFIDENCES
                # conf = float(box.conf.item())
                # label = results.names[int(box.cls.item())]
                # self.get_logger().info(f"{label}: {conf:.3f}")

                detection_msg.detections.append(bbox)

        # self.get_logger().info(f"Detected {len(detection_msg.detections)} objects in {inference_time:.3f}s")
        # for detection in detection_msg.detections:
        #     self.get_logger().info(f"  - {detection.class_name}: {detection.confidence:.2f}")

        self.detection_pub.publish(detection_msg)

        if self.debug_w_visualizer:
            annotator = Annotator(self.latest_image.copy())
            if results.boxes is not None:
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    cls = int(box.cls.item())
                    conf = float(box.conf.item())
                    label = f"{results.names[cls]} {conf:.2f}"
                    annotator.box_label(xyxy, label)

            debug_image = annotator.result()
            self.publish_debug_image(debug_image)

    def publish_debug_image(self, debug_image: NDArray[np.uint8]) -> None:
        debug_msg: Image = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = self.camera_frame
        self.debug_yolo_pub.publish(debug_msg)


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    try:
        yolo_node = YOLONode()
        rclpy.spin(yolo_node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
