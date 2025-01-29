#! /usr/bin/env python3

import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from vox_nav_msgs.action import ComputePathToPose

from scipy.spatial.transform import Rotation as R


class PlanExecutor(Node):
    def __init__(self):
        super().__init__('plan_executor')
        self.get_logger().info('Plan Executor Node Started')
        self.namespace = self.declare_parameter('namespace', 'atlas').value
        self.planner_id = self.declare_parameter('planner_id', 'vox_nav_planning::SE2Planner').value
        # we need a callback group to call backbacks from within callbacks
        self.reentrant_callback_group = ReentrantCallbackGroup()

        # subscriptions
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, f'/{self.namespace}/goal_pose', self.goal_pose_callback, 10, callback_group=self.reentrant_callback_group)

        # action clients
        self.compute_plan_action_topic = f'/{self.namespace}/compute_path_to_pose'
        self.compute_plan_action_client = ActionClient(
            self, ComputePathToPose, self.compute_plan_action_topic, callback_group=self.reentrant_callback_group)

    def plan_callback(self, msg):
        self.get_logger().info(f'Received Plan: {msg}')

    def goal_pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        quat = msg.pose.orientation
        rpy = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
        self.get_logger().info(
            f'Received Goal Pose: x = {pos.x}, y = {pos.y}, z = {pos.z}, roll = {rpy[0]}, pitch = {rpy[1]}, yaw = {rpy[2]}')

        print(f'Sending goal to planner on topic {self.compute_plan_action_topic}')

        self.send_planner_goal(msg)

    def send_planner_goal(self, goal_pose: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.planner_id = self.planner_id

        self.compute_plan_action_client.wait_for_server(timeout_sec=2.0)

        self.send_goal_future = self.compute_plan_action_client.send_goal_async(goal_msg, feedback_callback=self.plan_callback)

    def plan_callback(self, msg):
        self.get_logger().info(f'Received Plan: {msg}')


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    plan_executor = PlanExecutor()
    executor.add_node(plan_executor)

    try:
        rclpy.spin(node=plan_executor, executor=executor)
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')
    finally:
        print('Shutting down plan_executor...')
        plan_executor.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
