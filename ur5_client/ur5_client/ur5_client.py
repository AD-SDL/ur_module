#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from ur5_driver.ur5_driver import UR5 
from time import sleep


from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions  

class UR5ClientNode(Node):
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
        

        self.state = "UNKNOWN"

        self.connect_robot()
        # self.test()

        timer_period = 0.5  # seconds
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)

        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

   
        self.action_handler = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)

        self.description={}

    def test(self):
        
        for i in range(3):
            sleep(20)
            self.ur5.transfer(self.ur5.plate_exchange_1,self.ur5.plate_exchange_1)
        pass
        
    def connect_robot(self):
        
        try:
            self.ur5 = UR5()
        except Exception as err:
            self.get_logger().error(err)
        else:
            self.get_logger().info("UR5 connected")
            
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

            self.ur5.transfer(pos1, pos2)
            

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
    node = UR5ClientNode(NODE_NAME = NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()