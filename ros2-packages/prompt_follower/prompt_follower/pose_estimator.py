from __future__ import annotations

from typing import Optional

import numpy as np

import rclpy
from rclpy.time import Time

from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
)
from sensor_msgs.msg import CameraInfo

import tf2_ros

from tf2_geometry_msgs import do_transform_point, do_transform_pose_stamped
from tf_transformations import quaternion_from_euler
import math
from rclpy.node import Node

from .detector import Detection

class PoseEstimator:
    def __init__(
        self,
        node: Node,
        tf_buffer: tf2_ros.Buffer,
        target_frame: str = "map"
    ):
        self.node = node

        self.last_yaw = None
        self.last_spin_direction = 1
        
        self.tf_buffer = tf_buffer

        self.target_frame = target_frame

        self.camera_info: Optional[CameraInfo] = None

        self.node.get_logger().info(f'Created PoseEstimator object')
        self.node.get_logger().info(f'  target_frame: {target_frame}')

        self.last_point_map = None

    def set_camera_info(
        self,
        camera_info: CameraInfo,
    ):
        self.camera_info = camera_info

    def estimate_pose(
        self,
        detections: list[Detection],
        depth_image: np.ndarray,
        camera_frame: str,
    ) -> Optional[PoseStamped]:
        if self.camera_info is None:
            self.node.get_logger().warning(f'  Camera info not available')
            return None
        
        try:
            robot_pose = PoseStamped()
            robot_pose.pose.position.x = 0.0
            robot_pose.pose.position.y = 0.0
            robot_pose.pose.position.z = 0.0
            aux = quaternion_from_euler(
                0.0,
                0.0,
                0.0
            )
            robot_pose.pose.orientation.w = aux[3]
            robot_pose.pose.orientation.x = aux[0]
            robot_pose.pose.orientation.y = aux[1]
            robot_pose.pose.orientation.z = aux[2]


            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            robot_pose_map = do_transform_pose_stamped(
                robot_pose,
                transform,
            )
        except Exception:
            self.node.get_logger().error(f'  Exception when obtaining robot position in map')
            return None

        best_point_map = None
        best_distance = 10000000000.0
        best_detection = None

        for detection in detections:
            depth = self._estimate_depth(
                detection,
                depth_image,
            )
            if depth is None:
                continue

            self.node.get_logger().info(f'  Depth is {depth}')
            x, y, z = self._pixel_to_camera(
                detection.centroid[0],
                detection.centroid[1],
                depth,
            )
            self.node.get_logger().info(f'  x: {x}, y: {y}, z: {z}')
            point = PointStamped()

            point.header.frame_id = camera_frame
            point.header.stamp = Time().to_msg()

            point.point.x = x
            point.point.y = y
            point.point.z = z

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                point_map = do_transform_point(
                    point,
                    transform,
                )
            except Exception:
                self.node.get_logger().error(f'Exception')
                continue

            if self.last_point_map:
                distance = math.hypot(point_map.point.x - self.last_point_map.point.x, point_map.point.y - self.last_point_map.point.y)
            else:
                distance = 1.0 - detection.confidence
            
            if (distance < best_distance):
                best_point_map = point_map
                best_distance = distance
                best_detection = detection
        
        if not best_point_map:
            self.node.get_logger().warning(f'  No best point was obtained')
            return None
        
        pose = PoseStamped()

        pose.header = best_point_map.header

        pose.pose.position = best_point_map.point
        
        yaw, self.last_point_map = self._estimate_yaw(
            best_detection,
            depth_image,
            robot_pose_map,
            camera_frame
        )

        self.node.get_logger().info(f'  yaw: {yaw}')
        q = quaternion_from_euler(
            0.0,
            0.0,
            yaw,
        )
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.node.get_logger().info('cp')

        return pose

    def _estimate_depth(
        self,
        detection: Detection,
        depth_image: np.ndarray,
    ) -> Optional[float]:
        if detection.mask is not None:
            mask = detection.mask > 0.5

            values = depth_image[mask]
        else:
            x1, y1, x2, y2 = detection.bbox.astype(int)

            roi = depth_image[y1:y2, x1:x2]

            values = roi.flatten()
        values = values[values > 0]

        if len(values) == 0:
            return None
        
        depth = np.percentile(values, 5)

        return max(0.0, float(depth) / 1000.0 - self.node.get_parameter("goal_offset").value)

    def _estimate_yaw(
        self,
        detection: Detection,
        depth_image: np.ndarray,
        robot_pose_map: PoseStamped,
        camera_frame: str
    ):
        if detection.mask is not None:
            mask = detection.mask > 0.5

            values = depth_image[mask]
        else:
            x1, y1, x2, y2 = detection.bbox.astype(int)

            roi = depth_image[y1:y2, x1:x2]

            values = roi.flatten()
        values = values[values > 0]

        if len(values) == 0:
            return None
        
        depth = np.percentile(values, 5)

        depth = max(0.0, float(depth) / 1000.0)

        x, y, z = self._pixel_to_camera(
            detection.centroid[0],
            detection.centroid[1],
            depth,
        )
        point = PointStamped()

        point.header.frame_id = camera_frame
        point.header.stamp = Time().to_msg()

        point.point.x = x
        point.point.y = y
        point.point.z = z

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            point_map = do_transform_point(
                point,
                transform,
            )
        except Exception:
            self.node.get_logger().error(f'E  xception')
            return

        pose = PoseStamped()
        
        pose.header = point_map.header

        pose.pose.position = point_map.point

        yaw = math.atan2(
            pose.pose.position.y - robot_pose_map.pose.position.y,
            pose.pose.position.x - robot_pose_map.pose.position.x
        )

        if self.last_yaw is not None:
            dyaw = (yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi

            if abs(dyaw) > 0.02:
                if dyaw > 0:
                    self.last_spin_direction = 1
                else:
                    self.last_spin_direction = -1

        self.last_yaw = yaw

        return yaw, point_map

    def _pixel_to_camera(
        self,
        u: int,
        v: int,
        depth: float,
    ):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]

        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return x, y, z

    def get_spin_direction(self):
        return self.last_spin_direction