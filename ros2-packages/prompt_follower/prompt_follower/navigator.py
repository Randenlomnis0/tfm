from __future__ import annotations

from typing import Optional

import math

from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, Spin

from tf_transformations import euler_from_quaternion
from rclpy.node import Node

class Navigator:
    def __init__(
        self,
        node: Node,
        resend_threshold: float = 0.01
    ):
        self.node = node
        
        self.action_client = ActionClient(
            node,
            NavigateToPose,
            "navigate_to_pose"
        )
        self.spin_action_client = ActionClient(
            node,
            Spin,
            "spin"
        )
        self.node = node

        self.resend_threshold = resend_threshold

        self.last_goal: Optional[PoseStamped] = None

        self.node.get_logger().info(f'Created Navigator object')
        self.node.get_logger().info(f'  resend_threshold: {resend_threshold}')

        self.goal_handle = None

    def wait_until_active(self):
        self.action_client.wait_for_server()
        self.spin_action_client.wait_for_server()

    def goto(self, goal: PoseStamped):
        """
        Send a goal.
        """
        self.current_goal = goal

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.las
            self.node.get_logger().warning("  Goal rejected")
            return
        
        self.node.get_logger().info(f'  Goal accepted')

        self.goal_handle = goal_handle
        self.last_goal = self.current_goal

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status

        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("  Goal Reached")
        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().warning('  Goal Aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().warning('  Goal Aborted')
        elif status == GoalStatus.STATUS_UNKNOWN:
            self.node.get_logger().warning('  Goal Aborted')
        else:
            self.node.get_logger().warning(f"  Goal finished with status {status}")

    def spin(self, spin_direction):
        spin_msg = Spin.Goal()
        spin_msg.target_yaw = 0.15 * spin_direction

        future = self.spin_action_client.send_goal_async(spin_msg)
        future.add_done_callback(self._spin_goal_response_callback)

    def _spin_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().warning("  Spin Goal rejected")
            return
        
        self.node.get_logger().info(f'  Spin Goal accepted')

        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_result_callback)
    
    def _spin_result_callback(self, future):
        status = future.result().status

        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("  Spin Goal Reached")
        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().warning('  Spin Goal Aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().warning('  Spin Goal Cborted')
        elif status == GoalStatus.STATUS_UNKNOWN:
            self.node.get_logger().warning('  Spin Goal Aborted')
        else:
            self.node.get_logger().warning(f"  Spin Goal finished with status {status}")

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def is_navigating(self):
        return self.goal_handle is not None

    def same_goal(
        self,
        goal: PoseStamped,
    ) -> bool:
        if self.last_goal is None:
            return False
        dx = (
            goal.pose.position.x
            - self.last_goal.pose.position.x
        )
        dy = (
            goal.pose.position.y
            - self.last_goal.pose.position.y
        )
        d = math.hypot(dx, dy)

        q1 = [
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w
        ]
        e1 = euler_from_quaternion(q1)

        q2 = [
            self.last_goal.pose.orientation.x,
            self.last_goal.pose.orientation.y,
            self.last_goal.pose.orientation.z,
            self.last_goal.pose.orientation.w
        ]
        e2 = euler_from_quaternion(q2)

        return d < self.resend_threshold and abs((e1[2] - e2[2] + math.pi) % (2 * math.pi) - math.pi) < 0.05