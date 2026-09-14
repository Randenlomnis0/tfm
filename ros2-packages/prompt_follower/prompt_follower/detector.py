from dataclasses import dataclass
from typing import List, Optional

import time

import numpy as np
import cv2

from ultralytics import YOLO

from sensor_msgs.msg import Image
from rclpy.node import Node

@dataclass
class Detection:
    label: str
    confidence: float
    bbox: np.ndarray
    mask: Optional[np.ndarray]
    centroid: tuple[int, int]

class YOLOEDetector:
    def __init__(
        self,
        node: Node,
        model_path: str = "models/yoloe-26m-seg.pt",
        image_size: int = 640
    ):
        self.node = node

        self.image_pub1 = self.node.create_publisher(
            Image,
            "yolo/debug",
            10
        )

        self.image_pub2 = self.node.create_publisher(
            Image,
            "orin/debug",
            10
        )

        self.model = YOLO(model_path)

        self.image_size = image_size

        self.current_classes: List[str] = []

        self.node.get_logger().info(f'Created YOLOEDetector object')
        self.node.get_logger().info(f'  model_path: {model_path}')
        self.node.get_logger().info(f'  image_size: {image_size}')

    def set_classes(self, classes: List[str]):
        """
        Set detection vocabulary.
        """
        start = time.perf_counter()

        self.current_classes = classes
        self.model.set_classes(classes)

        self.node.get_logger().info(f'Changing classes took {time.perf_counter() - start:.6f} seconds')

    def detect(self, image: np.ndarray) -> tuple[List[Detection], np.ndarray]:
        """
        Returns every detection found in the image.
        """
        start = time.perf_counter()
        results = self.model.predict(
            source=image,
            imgsz=self.image_size,
            conf=self.node.get_parameter("confidence").value,
            verbose=False,
        )
        self.node.get_logger().info(f'Prediction took {time.perf_counter() - start:.6f} seconds')
        result = results[0]

        annotated = result.plot()
        
        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.height = annotated.shape[0]
        msg.width = annotated.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = annotated.shape[1] * annotated.shape[2]
        msg.data = annotated.tobytes()

        self.image_pub1.publish(msg)

        detections: List[Detection] = []

        if result.boxes is None:
            return [], image.copy()
        boxes = result.boxes
        masks = result.masks

        for i, box in enumerate(boxes):
            cls = int(box.cls.item())

            if self.current_classes and cls < len(self.current_classes):
                label = self.current_classes[cls]
            else:
                label = str(cls)
            conf = float(box.conf.item())

            xyxy = box.xyxy[0].cpu().numpy()
            xmin, ymin, xmax, ymax = xyxy.astype(int)

            cx = (xmin + xmax) // 2
            cy = (ymin + ymax) // 2

            mask = None

            if masks is not None:
                mask = masks.data[i].cpu().numpy()

                mask = cv2.resize(
                    mask,
                    (image.shape[1], image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            detections.append(
                Detection(
                    label=label,
                    confidence=conf,
                    bbox=xyxy,
                    mask=mask,
                    centroid=(cx, cy),
                )
            )
        bbox_img = image.copy()

        for i, detection in enumerate(detections):
            xmin, ymin, xmax, ymax = detection.bbox.astype(int)

            cv2.rectangle(
                bbox_img,
                (xmin, ymin),
                (xmax, ymax),
                (0, 255, 0),
                2,
            )

            cv2.putText(
                bbox_img,
                str(i),
                (xmin + 5, ymin + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.height = bbox_img.shape[0]
        msg.width = bbox_img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = bbox_img.shape[1] * bbox_img.shape[2]
        msg.data = bbox_img.tobytes()

        self.image_pub2.publish(msg)

        return detections, bbox_img

    @staticmethod
    def best_detection(
        detections: List[Detection]
    ) -> Optional[Detection]:
        """
        Choose the detection with the largest confidence.
        """
        if not detections:
            return None
        best = None
        best_conf = -1

        for det in detections:
            if det.confidence > best_conf:
                best_conf = det.confidence
                best = det
        return best