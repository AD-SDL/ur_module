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
   
    def __init__(self, TEMP_NODE_NAME = "ur_node") -> None:

        super().__init__(TEMP_NODE_NAME)
        
        self.ur = None
        self.IP = None
        self.node_name = None

        self._receive_launch_parameters()  

        action_name = self.node_name + '/wei_action'
        self._action_server = ActionServer(self, RobotAction, action_name, self.action_callback)

        self.get_logger().info("Listening for action calls over: " + action_name)
    
    def _receive_launch_parameters(self) -> None:
        """Receives launch parameters from the launch execution"""

        self.node_name = self.get_name()
        self.declare_parameter('ip', "164.54.116.129") # Declaring parameter 
        self.IP = self.get_parameter('ip').get_parameter_value().string_value   
    
    def _connect_robot(self) -> None:
        """Creates a connection with the robot over URx. This fuction utilizes the ur_driver package"""    
        try:
            self.ur = UR(self.IP)

        except Exception as err:
            self.get_logger().error(str(err))
        else:
            self.get_logger().info("ur connected")

    def action_callback(self, goal_handle) -> RobotAction.Result:
        self.get_logger().info('Executing goal...')
        self.get_logger().info(str(goal_handle.request.robot_goal))

        if self.IP != "None":  
            response = self._action_handle(goal = goal_handle)
        else:
            self._ros_driver_handle(goal_handle) #Use MoveIt

        # feedback = RobotAction.Feedback()

        # for i in range(20):
        #     feedback.robot_feedback = ' Busy ' + str(i) +' seconds'
        #     self.get_logger().info('Feedback:'+ (feedback.robot_feedback))

        #     goal_handle.publish_feedback(feedback)
        #     sleep(1)

        result = RobotAction.Result()
        result.robot_response = response
        return result

    def _action_handle(self, goal) -> str:
        
        robot_command = json.loads(goal.request.robot_goal)
        msg = None
        #TODO: Execute a robot state check and accept the action if robot is in ready state
        
        try:
            self._connect_robot()

            if "transfer" in robot_command:
                vars = robot_command.get("transfer")
                source_loc = vars.get("source")
                target_loc = vars.get("target")
                self.ur.transfer(source_loc, target_loc)     
      
            elif "run_urp_program" in robot_command:
                vars = robot_command.get("run_urp_program")
                local_urp_path = vars.get('local_urp_path', None)
                program_name = vars.get('program_name', None)
                output_log = self.ur.run_urp_program(transfer_file_path = local_urp_path, program_name = program_name)
                self.get_logger().warn(output_log)

            elif "pick_tool" in robot_command:
                vars = robot_command.get("pick_tool")
                home_loc = vars.get('home', None)
                tool_loc = vars.get('tool_loc', None)
                self.ur.pick_tool(home_loc, tool_loc)            

            elif "place_tool" in robot_command:
                vars = robot_command.get("pick_tool")
                home_loc = vars.get('home', None)
                tool_loc = vars.get('tool_loc', None)
                self.ur.place_tool(home_loc, tool_loc)            

            self.ur.ur.disconnect_ur()

        except Exception as er:
            msg = {-1:"Failed " + er}
            goal.abort()
        else:
            msg = {0:"Completed"}
            goal.succeed()
        finally:
            response = json.dumps(msg)
            return response   
        
    def _ros_driver_handle(self, goal):
        pass

def main(args=None) -> None:
    rclpy.init(args=args)

    ur_node = UrActionServer()

    rclpy.spin(ur_node)


if __name__ == '__main__':
    main()