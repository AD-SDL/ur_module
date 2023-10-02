import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String

from time import sleep
import socket
import json

from ur_interfaces.action import RobotAction

try:
    from ur_driver.ur_driver import UR 
except ImportError:
    print("UR driver package cannot be used!")

class UrActionServer(Node): #ACTION SERVER

    '''
    This is a stakeholder for the UR action server class. 
    '''
   
    def __init__(self, TEMP_NODE_NAME = "ur_action_server"):

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.ur = None
        self.IP = None
        self.receive_launch_parameters()    

        self.connect_robot()

        self.state = "UNKNOWN"
        self.robot_status = None
        self.action_flag = "READY"

        self._action_server = ActionServer(self, RobotAction, self.node_name + '/robot_action', self.action_callback)
   
    def receive_launch_parameters(self):
        
        self.declare_parameter('ip', '146.137.240.38')       # Declaring parameter 
        self.IP = self.get_parameter('ip').get_parameter_value().string_value     
    
    def connect_robot(self):
        """Creates a connection with the robot over URx. This fuction utilizes the ur_driver package"""    
        try:
            self.ur = UR(self.IP)

        except Exception as err:
            self.get_logger().error(str(err))
        else:
            self.get_logger().info("ur connected")

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