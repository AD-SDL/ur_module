#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions  

class MirClientNode(Node):
    '''
    The jointControlNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the peeler and a description of the peeler to the respective topics.
    '''
    def __init__(self, NODE_NAME = "UR5_Client_Node"):
        '''
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        
        print("UR5 is online") 

        self.state = "UNKNOWN"
        self.client = UR5()

        # self.client.initialize_robot()

        timer_period = 0.5  # seconds
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)

        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

   
        self.action_handler = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)
        self.description_handler = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)

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
        
        if request.action_handle=='move':
            pass

        if request.action_handle=='dock':
            pass
        
        


def main(args = None):

    NAME = "MiR_Client_Node"
    rclpy.init(args = args)  # initialize Ros2 communication
    node = MirClientNode(NODE_NAME = NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()