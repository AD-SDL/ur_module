import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from time import sleep
from ur_interfaces.action import RobotAction

class UrActionServer(Node): # ACTION SERVER

    '''
    The init function is neccesary for the URCLient class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    '''
   
    def __init__(self, TEMP_NODE_NAME = "Ur_Node"):

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.ur = None
        self.IP = None

        self._action_server = ActionServer(self, RobotAction, self.node_name + '/robot_action', self.action_callback)

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.get_logger().info(str(goal_handle.request.robot_goal))

        feedback = RobotAction.Feedback()

        for i in range(20):
            feedback.robot_feedback = ' Busy ' + str(i) +' seconds'
            self.get_logger().info('Feedback:'+ (feedback.robot_feedback))

            goal_handle.publish_feedback(feedback)
            sleep(1)

        goal_handle.succeed()
        result = RobotAction.Result()
        result.robot_response = goal_handle.request.robot_goal + " completed"
        return result

def main(args=None):
    rclpy.init(args=args)

    ur_node = UrActionServer()

    rclpy.spin(ur_node)


if __name__ == '__main__':
    main()