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
        

        self.ur = None
        self.IP = None
        self.node_name = None

        self._receive_launch_parameters()  

        self.state = "UNKNOWN"
        self.robot_status = None
        self.action_flag = "READY"

        action_name = self.node_name + '/robot_action'
        self._action_server = ActionServer(self, RobotAction, action_name, self.action_callback)

        self.get_logger().info("Listening for action calls over: " + action_name)
    
    def _receive_launch_parameters(self):
        
        self.node_name = self.get_name()
        self.declare_parameter('ip', "None")       # Declaring parameter 
        self.IP = self.get_parameter('ip').get_parameter_value().string_value   
    
    def _connect_robot(self):
        """Creates a connection with the robot over URx. This fuction utilizes the ur_driver package"""    
        try:
            self.ur = UR(self.IP)

        except Exception as err:
            self.get_logger().error(str(err))
        else:
            self.get_logger().info("ur connected")

    def action_callback(self, goal_handle):
        # self.get_logger().info('Executing goal...')
        self.get_logger().info(str(goal_handle.request.robot_goal))
        goal_handle.accepted()
        
        if self.IP != "None":  
            self._connect_robot()
            self._action_handle(goal = goal_handle)
        else:
            #Use MoveIt
            pass

        feedback = RobotAction.Feedback()

        for i in range(20):
            feedback.robot_feedback = ' Busy ' + str(i) +' seconds'
            self.get_logger().info('Feedback:'+ (feedback.robot_feedback))

            goal_handle.publish_feedback(feedback)
            sleep(1)

    
    def _action_handle(self, goal):
        
        robot_command = json.loads(goal.request.robot_goal)
        result = RobotAction.Result()

        if "transfer" in robot_command.keys():
            self.get_logger().info("Executing a transfer")
            vars = robot_command.get("transfer")
            source_loc = vars.get("source")
            target_loc = vars.get("target")
        
        try:
            self.ur.transfer(source_loc, target_loc)            
        except Exception as er:
            msg = {-1:"Transfer failed"}
            goal.aborted()
            result.robot_response = json.dumps(msg)
        else:
            msg = {0:"Transfer completed"}
            goal.succeed()
            result.robot_response = json.dumps(msg)
        finally:
            return result   


def main(args=None):
    rclpy.init(args=args)

    ur_node = UrActionServer()

    rclpy.spin(ur_node)


if __name__ == '__main__':
    main()