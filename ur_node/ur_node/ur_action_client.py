#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# from ur_interfaces.action import RobotAction
from wei_interfaces.action import WeiAction
import json

class UrActionClient(Node): # ACTION CLIENT

    def __init__(self, TEMP_NODE_NAME = "ur_node") -> None:

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.ur = None
        self.IP = None
        self.goal = None
        self._action_client = ActionClient(self, WeiAction, self.node_name + '/wei_action')

    def send_goal(self, robot_goal) -> None:
        self.goal = json.dumps(robot_goal)
        
        goal_msg = WeiAction.Goal()
        print(goal_msg)
        goal_msg.robot_goal = self.goal
        print(goal_msg)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.get_logger().info("Action started: " + self.goal)
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.robot_response))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback:' + feedback.robot_feedback)

def main(args=None):
    rclpy.init(args=args)

    action_client = UrActionClient()
    # goal = {"transfer":{"source":[1,1,1],"target":[1,1,1]}}
    goal = {"pick_tool":{"home":[1,1,1],"tool_loc":[1,1,1]}}

    action_client.send_goal(goal)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()