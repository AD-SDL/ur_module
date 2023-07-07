#! /usr/bin/env python3

import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String

from ur_driver.ur_driver import UR 
# import ur_driver.ur_driver.ur_tools
from time import sleep
import socket
import json

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions  

class URClient(Node):
    '''
    The init function is neccesary for the URCLient class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    '''
   
    def __init__(self, TEMP_NODE_NAME = "Ur_Client_Node"):

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.ur = None
        self.IP = None
        self.receive_launch_parameters()    
        
        self.get_logger().info("Received IP: " + str(self.IP))
        self.get_logger().info("Tools: " + "Gripper: " + str(self.gripper))

        self.connect_robot()

        self.state = "UNKNOWN"
        self.robot_status = None
        self.action_flag = "READY"
        
        action_cb_group = ReentrantCallbackGroup()
        state_refresher_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()


        state_publisher_period = 0.5  # seconds
        self.state_refresher_period = state_publisher_period + 1.0  # seconds

        self.statePub = self.create_publisher(String, self.node_name + '/state', 10)
        self.stateTimer = self.create_timer(state_publisher_period, self.stateCallback, callback_group = state_cb_group)
        
        self.StateRefresherTimer = self.create_timer(self.state_refresher_period, callback = self.stateRefresherCallback, callback_group = state_refresher_cb_group)

        self.action_handler = self.create_service(WeiActions, self.node_name + "/action_handler", self.actionCallback, callback_group=action_cb_group)

        self.description={}
        self.descriptionSrv = self.create_service(WeiDescription, self.node_name + "/description_handler", self.descriptionCallback, callback_group=description_cb_group)
    
    def receive_launch_parameters(self):
        
        self.declare_parameter('ip', '146.137.240.38')       # Declaring parameter 
        self.IP = self.get_parameter('ip').get_parameter_value().string_value     
        
        self.declare_parameter('gripper', False)       # Declaring parameter 
        self.gripper = self.get_parameter('gripper').get_parameter_value().bool_value
       
        self.declare_parameter('vacuum_gripper', False)       # Declaring parameter 
        self.vacuum_gripper = self.get_parameter('vacuum_gripper').get_parameter_value().bool_value

        self.declare_parameter('screwdriver', False)       # Declaring parameter 
        self.screwdriver = self.get_parameter('screwdriver').get_parameter_value().bool_value

        self.declare_parameter('pipette_pv', "None")       # Declaring parameter 
        self.pipette_pv = eval(self.get_parameter('pipette_pv').get_parameter_value().string_value )
        
        self.declare_parameter('tool_changer_pv', "None")       # Declaring parameter 
        self.tool_changer_pv = eval(self.get_parameter('tool_changer_pv').get_parameter_value().string_value)

        self.declare_parameter('camera_pv', "None")       # Declaring parameter 
        self.camera_pv = eval(self.get_parameter('camera_pv').get_parameter_value().string_value)

    def connect_robot(self):
        
        try:
            self.ur = UR(self.IP, gripper = self.gripper, tool_changer_pv = self.tool_changer_pv, pipette_pv =self.pipette_pv, camera_pv = self.camera_pv)
        except Exception as err:
            self.get_logger().error(str(err))
        else:
            self.get_logger().info("ur connected")

    def stateRefresherCallback(self):
        """ Refreshes the robot states if robot cannot update the state parameters automatically because it is not running any jobs
       
         Parameters:
        -----------
            None
        Returns
        -------
            None
        """  
        err = None

        try:

            if self.action_flag == "READY": #Only refresh the state manualy if robot is not running a job.
                self.ur.get_overall_robot_status()


        except UnboundLocalError as local_var_err:
            err = local_var_err

        except TimeoutError as time_err:
            err = time_err

        except AttributeError as attribute_err:
            err = attribute_err
            self.get_logger().warn("Trying to connect again! IP: " + self.ip + " Port:" + str(self.port))
            self.connect_robot()

        finally:
            if err:
                self.state = "ERROR"
                self.get_logger().error(str(err))
        
    def stateCallback(self):
        '''
        Publishes the ur state to the 'state' topic. 
        '''
        msg = String()

        #BUG: FIX EXEPCTION HANDLING TO HANDLE SOCKET ERRORS AND OTHER ERRORS SEPERATLY

        try:
            self.movement_state = self.ur.get_movement_state()

        except socket.error as err:
            self.get_logger().error("ROBOT IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "ur CONNECTION ERROR"
        except Exception as general_err:
            self.get_logger().error(str(general_err))
            
        if self.state != "ur CONNECTION ERROR":

            if self.ur.remote_control_status == False:
                self.get_logger().error("Please put the UR into remote mode using the Teach Pendant")

            elif "RUNNING" not in self.ur.robot_mode or "NORMAL" not in self.ur.safety_status or self.state == "ERROR":
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.get_logger().error("Robot_Mode: " + self.ur.robot_mode + " Safety_Status: " + self.ur.safety_status)
                self.action_flag = "READY"
                self.get_logger().warn("Trying to clear the error messages")
                # self.ur.initialize()
                self.action_flag = "UNKOWN"

            elif self.state == "COMPLETED" and self.action_flag == "BUSY":
                self.state = "COMPLETED"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.action_flag = "READY"

            elif self.movement_state == "BUSY" or self.action_flag == "BUSY":
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.ur.robot_mode == "RUNNING" and self.ur.safety_status == "NORMAL" and self.movement_state == "READY" and self.action_flag == "READY":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            else:
                self.state = "UNKOWN"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().warn(msg.data)
        else: 
            msg = String()
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! IP: " + self.IP)
            # self.connect_robot()

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''
        
        if request.action_handle=='transfer':
            self.action_flag = "BUSY"
            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            if 'pos1' not in vars.keys() or 'pos2' not in vars.keys():
                self.get_logger().error('vars wrong')
                return 

            pos1 = vars.get('pos1')
            self.get_logger().info(str(pos1))
            pos2 = vars.get('pos2')
            self.get_logger().info(str(pos2))

            try:
                self.ur.transfer(pos1, pos2)            
            except Exception as er:
                response.action_response = -1
                response.action_msg = "Transfer failed"
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg = "Transfer successfully completed"
                self.state = "COMPLETED"
            finally:
                return response
            
        elif request.action_handle == 'run_urp_program':
            self.action_flag = "BUSY"

            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            local_urp_path = vars.get('local_urp_path', None)
            self.get_logger().info(str(local_urp_path))
            program_name = vars.get('program_name', None)
            self.get_logger().info(str(program_name))

            if not program_name: 
                self.get_logger().err("Program name is not provided!")
                return
            
            try:
                output_log = self.ur.run_urp_program(transfer_file_path = local_urp_path, program_name = program_name)
            except Exception as er:
                response.action_response = -1
                response.action_msg = output_log["output_msg"] + output_log["output_log"]
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg = output_log["output_msg"] + output_log["output_log"]
                self.state = "COMPLETED"
            finally:
                return response
            #BUG: Action response never sent back

def main(args = None):

    rclpy.init(args=args)  # initialize Ros2 communication

    try:
        ur_client = URClient()
        executor = MultiThreadedExecutor()
        executor.add_node(ur_client)

        try:
            ur_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            ur_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            ur_client.ur.disconnect_ur()
            ur_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()