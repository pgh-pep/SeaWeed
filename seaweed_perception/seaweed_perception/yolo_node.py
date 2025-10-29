#!/usr/bin/env python3
"""Detect the objects in the image from the camera stream using ultralytics YOLO model."""

from typing import Optional, List
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO

from seaweed_interfaces.msg import BoundingBox, Detection


class YOLONode(Node):
    """ROS2 node that uses YOLO model to detect objects in images from a camera stream."""

    def __init__(self):
        super().__init__("yolo_node")
        self.subscription = self.create_subscription(
            Image, "/wamv/sensors/cameras/camera_sensor/image_raw", self.camera_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # load a model
        self.model = YOLO("models/best.pt")  # load a pretrained model

        # create a publisher
        self.publisher_ = self.create_publisher(Detection, "detections", 10)

    def camera_callback(self, msg: Image):
        self.get_logger().info("Receiving video frame")
        # convert ROS Image message to OpenCV image
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return  # Exit the callback if image conversion fails

        # Get image dimensions
        height, width = cv_image.shape[:2]

        # perform inference on the image
        start_time = time.time()
        results = self.model(cv_image)
        inference_time = time.time() - start_time

        # since we do inference on a single image frame, we need only get the first result
        single_result = results[0]

        # Create Detection message
        detection_msg = Detection()

        # Set header with timestamp
        detection_msg.header = Header()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "camera_frame"

        # Set image information
        detection_msg.image_width = width
        detection_msg.image_height = height
        detection_msg.inference_time = inference_time

        # Process detections
        detection_msg.detections = []

        if single_result.boxes is not None and len(single_result.boxes) > 0:
            for box in single_result.boxes:
                # Create BoundingBox message
                bbox = BoundingBox()

                # Extract box coordinates (xyxy format)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Convert to top-left corner and width/height
                bbox.x = float(x1)
                bbox.y = float(y1)
                bbox.width = float(x2 - x1)
                bbox.height = float(y2 - y1)

                # Normalized coordinates (center and size)
                bbox.x_normalized = float((x1 + x2) / 2 / width)
                bbox.y_normalized = float((y1 + y2) / 2 / height)
                bbox.width_normalized = float((x2 - x1) / width)
                bbox.height_normalized = float((y2 - y1) / height)

                # Class information
                bbox.class_id = int(box.cls.item())
                bbox.class_name = single_result.names[bbox.class_id]
                bbox.confidence = float(box.conf.item())

                detection_msg.detections.append(bbox)

        # Log detection info
        self.get_logger().info(f"Detected {len(detection_msg.detections)} objects in {inference_time:.3f}s")
        for detection in detection_msg.detections:
            self.get_logger().info(f"  - {detection.class_name}: {detection.confidence:.2f}")

        # Publish the detection message
        self.publisher_.publish(detection_msg)

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
