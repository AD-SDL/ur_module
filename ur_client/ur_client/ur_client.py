#! /usr/bin/env python3

import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String

from ur_driver.ur_driver import UR 
from time import sleep
import socket
import json

from ur_interfaces.action import RobotAction

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
        self._receive_launch_parameters()    
        
        self.get_logger().info("Received IP: " + str(self.IP))
        self.get_logger().info("Tools: " + "Gripper: " + str(self.gripper))
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self._connect_robot()


    def _receive_launch_parameters(self):
        
        self.declare_parameter('ip', '146.137.240.38')       # Declaring parameter 
        self.IP = self.get_parameter('ip').get_parameter_value().string_value     


    def _connect_robot(self):
        
        try:
            self.ur = UR(self.IP)

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
            
        if self.state != "UR CONNECTION ERROR":

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

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''
        self.action_flag = "BUSY"
        sleep(self.state_refresher_period + 0.1)

        if request.action_handle=='transfer':
            self.action_flag = "BUSY"
            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            if 'pos1' not in vars.keys() or 'pos2' not in vars.keys():
                self.get_logger().error('vars wrong')
                return 

            pos1 = vars.get('pos1')
            pos2 = vars.get('pos2')
            # self.get_logger().info(str(pos1), str(pos2))

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

            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            local_urp_path = vars.get('local_urp_path', None)
            program_name = vars.get('program_name', None)
            # self.get_logger().info(str(local_urp_path), str(program_name))

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

        elif request.action_handle == 'pick_tool':

            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            home_loc = vars.get('home', None)
            tool_loc = vars.get('tool_loc', None)
            # self.get_logger().info(str(home_loc),str(tool_loc))

            if not home_loc or tool_loc:
                response.action_response = -1
                response.action_msg = "Transfer failed"
                self.state = "ERROR"
                return response
            
            try:
                self.ur.pick_tool(home_loc, tool_loc)            
            except Exception as er:
                response.action_response = -1
                response.action_msg = "Pick tool failed"
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg = "Pick tool successfully completed"
                self.state = "COMPLETED"
            finally:
                return response 
            
        elif request.action_handle == 'place_tool':

            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            home_loc = vars.get('home', None)
            tool_loc = vars.get('tool_loc', None)
            # self.get_logger().info(str(home_loc),str(tool_loc))

            if not home_loc or tool_loc:
                response.action_response = -1
                response.action_msg = "Transfer failed"
                self.state = "ERROR"
                return response
            
            try:
                self.ur.place_tool(home_loc, tool_loc)            
            except Exception as er:
                response.action_response = -1
                response.action_msg = "Place tool failed"
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg = "Place tool successfully completed"
                self.state = "COMPLETED"
            finally:
                return response 
            
        elif request.action_handle == 'create_sample':

            vars = json.loads(request.vars)
            self.get_logger().info(str(vars))

            home_loc = vars.get('home', None)
            tip_loc = vars.get('tip_loc', None)
            sample_loc = vars.get('sample_loc', None)

            self.get_logger().info(str(home_loc), str(tip_loc), str(sample_loc))
   
            if not home_loc or not tip_loc or not sample_loc:
                response.action_response = -1
                response.action_msg = "Transfer failed"
                self.state = "ERROR"
                return response
            
            try:
                self.ur.create_sample(home_loc, tip_loc, sample_loc)            
            except Exception as er:
                response.action_response = -1
                response.action_msg = "Create sample failed"
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg = "Create sample successfully completed"
                self.state = "COMPLETED"
            finally:
                return response 
            
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
            ur_client.ur.ur_connection.disconnect_ur()
            ur_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()