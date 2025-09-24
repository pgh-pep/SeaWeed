#!/usr/bin/env python3
"""Detect the objects in the image from the camera stream using ultralytics YOLO model."""

from typing import Optional, List
import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO


class YOLONode(Node):
    """ROS2 node that uses YOLO model to detect objects in images from a camera stream."""

    def __init__(self):
        super().__init__("yolo_node")
        self.subscription = self.create_subscription(
            Image, "/wamv/sensors/cameras/camera_sensor/image_raw", self.camera_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # load a model
        self.model = YOLO("models/yolo11n.pt")  # load a pretrained model

    def camera_callback(self, msg: Image):
        self.get_logger().info("Receiving video frame")
        # convert ROS Image message to OpenCV image
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return  # Exit the callback if image conversion fails

        # perform inference on the image
        results = self.model(cv_image)
        # since we do inference on a single image, we can only get the first result
        single_result = results[0]
        labels = [single_result.names[cls.item()] for cls in single_result.boxes.cls.int()]
        self.get_logger().info(f"Detected labels: {labels}")
        # show results on the image
        cv_image = single_result.plot()
        # display the image with detections

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
