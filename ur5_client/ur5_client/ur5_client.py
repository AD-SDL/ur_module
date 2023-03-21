#! /usr/bin/env python3

import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String

from ur5_driver.ur5_driver import UR5 
from time import sleep

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions  

class UR5Client(Node):
    '''
    The jointControlNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the peeler and a description of the peeler to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "UR5_Client_Node"):
        '''
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.state = "UNKNOWN"

        timer_period = 0.5  # seconds
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.statePub = self.create_publisher(String, self.node_name + '/state', 10)

        self.stateTimer = self.create_timer(timer_period, self.stateCallback)
   
        self.action_handler = self.create_service(WeiActions, self.node_name + "/action_handler", self.actionCallback)

        self.description={}


    def stateCallback(self):
        '''
        Publishes the peeler state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.state = "READY"

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
            self.state = "BUSY"
            self.stateCallback()
            vars = eval(request.vars)
            print(vars)

            if 'pos1' not in vars.keys() or 'pos2' not in vars.keys():
                print('vars wrong')
                return 

            pos1 = vars.get('pos1')
            print(pos1)
            pos2 = vars.get('pos2')
            print(pos2)

            ur5 = UR5()
            ur5.transfer(pos1, pos2)
            

        self.state = "COMPLETED"

        return response


    def moveJCallback(self, request, response):
        '''
        The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.
        '''

        # self.state = "BUSY"
        # self.stateCallback()

        # pos = request.joint_positions                                                       # Joint position taken from list given within request 

        # print(pos)
        # pos1 = request.joint_positions[0:6]
        # print(pos1)
        # pos2 = request.joint_positions[6:12]
        # print(pos2)
        # ur5 = UR5()
        # ur5.transfer(pos1, pos2)
        # self.state = "COMPLETED"

        # return response
        pass

def main(args = None):

    NAME = "UR5_Client_Node"
    rclpy.init(args = args)  # initialize Ros2 communication
    node = UR5Client(NODE_NAME = NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication
    rclpy.init(args=args)       # initialize Ros2 communication

    try:
        peeler_client = UR5Client()
        executor = MultiThreadedExecutor()
        executor.add_node(peeler_client)

        try:
            peeler_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            peeler_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            peeler_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()