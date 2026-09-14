#!/usr/bin/env python3

from __future__ import annotations

import rclpy

from rclpy.node import Node

from cv_bridge import CvBridge

from std_msgs.msg import String

from sensor_msgs.msg import (
    Image,
    CameraInfo,
)
from geometry_msgs.msg import PoseStamped

import tf2_ros

from tf_transformations import euler_from_quaternion

import cv2

import base64
from ollama import Client
import time

from .detector import YOLOEDetector
from .pose_estimator import PoseEstimator
from .navigator import Navigator

class PromptFollower(Node):
    def __init__(self):
        super().__init__("prompt_follower")

        self.get_logger().info(f'Starting {self.get_name()}')

        self.declare_parameter("orin", "orin-no")
        self.declare_parameter("mode", "goto")
        self.declare_parameter("confidence", 0.10)
        self.declare_parameter("goal_offset", 2.0)
        self.declare_parameter("process_rate", 1.0 / 2.0)       # static

        process_rate = self.get_parameter(
            "process_rate").value
        self.get_logger().info(f'process_rate: {process_rate}')

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )
        self.detector = YOLOEDetector(
            self
        )
        self.pose_estimator = PoseEstimator(
            self,
            self.tf_buffer
        )
        self.navigator = Navigator(
            self
        )

        self.get_logger().info(f'Waiting for nav to be active')
        self.navigator.wait_until_active()
        self.get_logger().info(f'Nav is active!')

        self.detected = False
        self.current_detailed_prompt = None
        self.current_prompt = None
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_frame = None
        self.goal_sent = False
        self.cnt = 0

        self.get_logger().info(f'Subscribing to /prompt')
        self.create_subscription(
            String,
            "/prompt",
            self.prompt_callback,
            10,
        )
        self.get_logger().info(f'Subscribing to /j100_0562/D435_1/color/image_raw')
        self.create_subscription(
            Image,
            "/j100_0562/D435_1/color/image_raw",
            self.rgb_callback,
            10,
        )
        self.get_logger().info(f'Subscribing to /j100_0562/D435_1/aligned_depth_to_color/image_raw')
        self.create_subscription(
            Image,
            "/j100_0562/D435_1/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )
        self.get_logger().info(f'Subscribing to /j100_0562/D435_1/aligned_depth_to_color/camera_info')
        self.create_subscription(
            CameraInfo,
            "/j100_0562/D435_1/aligned_depth_to_color/camera_info",
            self.camera_info_callback,
            10,
        )
        self.get_logger().info(f'Publishing to /prompt_goal')
        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/prompt_goal",
            10,
        )

        self.timer = self.create_timer(
            1.0 / process_rate,
            self.process_frame,
        )

        self.get_logger().info(
            "Prompt follower ready."
        )

    def prompt_callback(self, msg: String):
        prompt = msg.data.strip()
        self.get_logger().info(f'Received new prompt: {prompt}')

        if prompt == "":
            return

        self.detected = False
        
        self.current_detailed_prompt, self.current_prompt = prompt.split(':')

        self.goal_sent = False

        self.detector.set_classes([self.current_prompt])

        self.get_logger().info(f'Tracking "{self.current_detailed_prompt}" and "{self.current_prompt}"')

    def rgb_callback(self, msg: Image):
        self.camera_frame = msg.header.frame_id

        self.latest_rgb = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.pose_estimator.set_camera_info(msg)

    def process_frame(self):
        self.get_logger().info(f'Processing frame')
        if self.current_prompt is None:
            return
        if self.latest_rgb is None:
            return
        if self.latest_depth is None:
            return
        if self.camera_frame is None:
            return
        if self.goal_sent and self.get_parameter("mode").value == "goto":
            self.get_logger().info(f'  Goal already sent')
            return
        self.get_logger().info(f'  Goal ready to be sent')
        relevant_depth = self.latest_depth
        detections, bbox_img = self.detector.detect(
            self.latest_rgb
        )
        self.get_logger().info(f'Detected {len(detections)} objects')
        if len(detections) == 0:
            self.cnt += 1
            if self.cnt >= 10:
                self.get_logger().warning(f'  Spinning because no object was detected in the last few frames')
                self.navigator.spin(self.pose_estimator.get_spin_direction())
            return
        self.cnt = 0
        best = self.detector.best_detection(
            detections
        )
        if best is None:
            self.get_logger().warning(f'  No best detection was found')
            return

        chosen_detections = list()
        for detection in detections:
            # if detection.confidence >= 0.8 * best.confidence:
            chosen_detections.append(detection)

        self.get_logger().info(f'  Chosen {len(chosen_detections)} detections')

        if (self.get_parameter("orin").value == "orin-yes") and (self.current_detailed_prompt is not None) and (not self.detected):
            success, buffer = cv2.imencode(".png", bbox_img)
            if success:
                img_base64 = base64.b64encode(buffer).decode("utf-8")

                selected_idx = self.select_bbox_smart(img_base64, self.current_detailed_prompt)
                
                if 0 <= selected_idx < len(chosen_detections):
                    self.detected = True

                    chosen_detections = [chosen_detections[selected_idx]]
                else:
                    return
            else:
                return

        self.get_logger().info(f'  Estimating pose')

        pose = self.pose_estimator.estimate_pose(
            chosen_detections,
            relevant_depth,
            self.camera_frame,
        )

        if pose is None:
            self.get_logger().warning(f'  No pose was estimated')
            return
        
        self.get_logger().info(f'  Publishing goal')

        self.goal_pub.publish(pose)

        if self.get_parameter("mode").value == "goto":
            self._handle_goto(pose)
        elif self.get_parameter("mode").value == "follow":
            self._handle_follow(pose)

    def _handle_goto(self, pose: PoseStamped):
        if self.goal_sent:
            return

        if self.navigator.is_navigating():
            self.navigator.cancel()
        
        self.navigator.goto(pose)

        self.goal_sent = True

        self.get_logger().info(f'[GOTO] Sent goal: {self.current_prompt}')

    def _handle_follow(self, pose: PoseStamped):
        self.goal_sent = True
        if self.navigator.is_navigating():
            if self.navigator.last_goal is not None:
                if self.navigator.same_goal(pose):
                    self.get_logger().warning(f'  Goal was too similar to previous goal')
                    return
            self.navigator.cancel()
        self.navigator.goto(pose)
    
    def select_bbox_smart(self, img_base64, target_description):
        orin_ip = "http://192.168.8.190:11434"

        client = Client(host=orin_ip)

        inicio = time.perf_counter()

        prompt = f"""
You are a visual bounding-box selection model.

You will receive one image containing multiple candidate bounding boxes. Each bounding box is drawn on the image and labeled with a numerical index on the inside of their upper-left corner. The index identifies that specific bounding box.

Your task is to find the single bounding box that best matches this target description:

TARGET:
{target_description}

Instructions:
- Inspect the visual content inside every labeled bounding box.
- Select only from the bounding boxes visibly present in the image.
- Match the target description based on visual appearance and meaning, not on the numerical index, box position, or nearby text.
- Choose the box containing the most direct and specific match.
- If multiple boxes match, choose the one that best satisfies the target description, preferably the most complete and clearly visible instance.
- Do not select a box merely because it is related to the target; select it only if the target itself, or the requested visual concept, is present in the box.
- If no bounding box clearly matches the target, output -1.
- Treat the numerical labels as identifiers only, not as part of the image content.
- If the target is present but has no matching bounding box, output -1.

Output requirements:
- Output exactly one integer.
- Output only the index corresponding to the selected bounding box, as it appears on the image, or -1.
- Do not output explanations, words, punctuation, formatting, or additional text.
    """

        self.get_logger().info("Sending request to Orin")

        models = [
            "qwen3.8:27b",
            "qwen3.6:35b",
            "qwen3.6:27b",
            "qwen3.5:9b",
            "gemma4:31b-it-q8_0",
            "gemma4:31b",
            "qwen3-vl:4b-instruct",
            "minicpm-v4.6",
            "minicpm-v4.5:8b"
        ]

        response = client.generate(
            model=models[0],
            prompt=prompt,
            keep_alive="1h",
            images=[img_base64],
            think='false',
            options={
                "temperature": 0.0,
                "num_predict": 150,
                "num_ctx": 1024,
                "top_k": 1
            }
        )

        fin = time.perf_counter()
        tiempo_total = fin - inicio

        self.get_logger().info(f"Total response time: {tiempo_total:.2f}s")

        raw_output = response['response']

        self.get_logger().info(f"Output: {raw_output}")

        if 'eval_duration' in response and 'eval_count' in response:
            tokens = response['eval_count']
            segundos_eval = response['eval_duration'] / 1e9
            tokens_por_segundo = tokens / segundos_eval if segundos_eval > 0 else 0
            
            self.get_logger().info(f"Generated tokens: {tokens}")
            self.get_logger().info(f"Token generation speed: {tokens_por_segundo:.2f} tokens/s")

        try:
            ans = int(raw_output)
            return ans
        except (ValueError, TypeError):
            self.get_logger().info(f"Output wasn't parseable as a string")
            return -1


def main(args=None):
    rclpy.init(args=args)

    node = PromptFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()