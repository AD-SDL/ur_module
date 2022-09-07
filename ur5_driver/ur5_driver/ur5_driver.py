import rclpy
from rclpy.node import Node
import threading

from builtin_interfaces.msg import Duration
from multiprocessing.connection import wait

import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


# import ur5_driver.robotiq_gripper as robotiq_gripper
import robotiq_gripper
from urx import Robot
from copy import deepcopy


class UR5(Node):
    commandLock = threading.Lock()

    def __init__(self):
        super().__init__(node_name= "ur5_driver")


        # ARM SETUP:
        ur_robot_ip = "192.168.1.102"
        i = 1
        while True:
            try:
                self.arm = Robot(ur_robot_ip)
                time.sleep(0.2)
                print('Successful arm connection on attempt #{}'.format(i))
                break
            except:
                print('Failed attempt #{}'.format(i))
                i+=1

        self.acceleration = 1.0
        self.velocity = 1.0

        self.home_pos = (0.0, -0.200, 0.59262, 2.247, 2.196, 0.0)


        # GRIPPER SETUP:
        print('Creating gripper...')
        self.gripper = robotiq_gripper.RobotiqGripper()
        print('Connecting to gripper...')
        
        self.gripper.connect(ur_robot_ip, 63352)
        if self.gripper.is_active():
            print('Gripper already active')
        else:
            print('Activating gripper...')
            self.gripper.activate()
        

        self.gripper_close = 110 # 0-255 (255 is closed)
        self.griper_open = 0
        self.gripper_speed = 150 # 0-255
        self.gripper_force = 0 # 0-255

        print('Opening gripper...')
        self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)


    def change_gripper_at_pos(self, goal, new_gripper_pos = 0):
        '''Publish trajectories to move to above goal, move down to goal, move to new gripper position, and move back to above goal'''
        self.commandLock.acquire()


        above_goal = deepcopy(goal)
        above_goal[2]+=0.20


        print('Moving to above goal position')
        self.arm.movel(above_goal, self.acceleration, self.velocity)


        print('Moving to goal position')
        self.arm.movel(goal, self.acceleration, self.velocity)


        print('Closing gripper')
        self.gripper.move_and_wait_for_pos(new_gripper_pos, self.gripper_speed, self.gripper_force)


        print('Moving back to above goal position')
        self.arm.movel(above_goal, self.acceleration, self.velocity)

        print('Moving back to home position')
        self.arm.movel(self.home_pos, self.acceleration, self.velocity)

        self.commandLock.release()


    def pick_up(self, pick_goal):
        '''Pick up from first goal position'''
        self.change_gripper_at_pos(pick_goal, self.gripper_close)
    

    def put_down(self, put_goal):
        '''Put down at second goal position'''
        self.change_gripper_at_pos(put_goal, self.griper_open)


    def transfer(self, pos1, pos2):

            self.pick_up(pos1)
            self.put_down(pos2)
            print('Finished transfer')


# def main(args=None):
#     rclpy.init(args=args)

#     publisher_joint_trajectory = PublisherJointTrajectory()

#     rclpy.spin(publisher_joint_trajectory)
#     publisher_joint_trajectory.destroy_node()
#     rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init(args=None)
    pos1= [-0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    pos2= [0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    robot = UR5()
    # rclpy.spin(robot)
    robot.transfer(pos1,pos2)
    robot.transfer(pos2,pos1)
    robot.arm.close()
    robot.destroy_node()
    rclpy.shutdown()
    # main()
    print('end')