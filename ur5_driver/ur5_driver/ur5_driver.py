import rclpy
from rclpy.node import Node
import threading

from builtin_interfaces.msg import Duration
from multiprocessing.connection import wait

import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


import ur5_driver.robotiq_gripper as robotiq_gripper


class UR5():
    commandLock = threading.Lock()

    def __init__(self):
        

        controller_name = "joint_trajectory_controller"
        self.joints = ["shoulder_pan_joint", 
                        "shoulder_lift_joint", 
                        "elbow_joint", 
                        "wrist_1_joint", 
                        "wrist_2_joint", 
                        "wrist_3_joint"
                        ]
        self.check_starting_point = False
        self.starting_point = {}
        self.gripper_close = 38
        self.griper_open = 0
        self.joint_limiter = {"shoulder_pan_joint": [-1.6,-1.5], 
                            "shoulder_lift_joint": [-0.1,0.1], 
                            "elbow_joint": [-2.3,-2.2], 
                            "wrist_1_joint": [-0.9,-0.8], 
                            "wrist_2_joint": [1.5,1.6],
                            "wrist_3_joint": [-0.1,0.1]
                            }
        self.goals = []

        ur_robot_ip = "192.168.1.102" 
        print('Creating gripper...')
        self.gripper = robotiq_gripper.RobotiqGripper()
        print('Connecting to gripper...')
        
        self.gripper.connect(ur_robot_ip, 63352)
        print('Activating gripper...')
        self.gripper.activate()
        print('Opening gripper...')

        self.gripper.move_and_wait_for_pos(0, 255, 255)


        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.positions_published = False

        # if self.joints is None or len(self.joints) == 0:
        #     raise Exception('"joints" parameter is not set!')

        # starting point stuff
        # if self.check_starting_point:
            # declare nested params
            # for name in self.joints:
                # param_name_tmp = "starting_point_limits" + "." + name
                # self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                # self.starting_point[name] = self.get_parameter(param_name_tmp).value

            # for name in self.joints:
            #     if len(self.starting_point[name]) != 2:
            #         raise Exception('"starting_point" parameter is not set correctly!')
            # self.joint_state_sub = self.create_subscription(
            #     JointState, "joint_states", self.joint_state_callback, 10
            # )
        # initialize starting point status
        # self.starting_point_ok = not self.check_starting_point

        # self.joint_state_msg_received = False

        # Read all positions from parameters
        # for name in goal_names:
        #     self.declare_parameter(name)
        #     goal = self.get_parameter(name).value
        #     if goal is None or len(goal) == 0:
        #         raise Exception(f'Values for goal "{name}" not set!')

        #     float_goal = []
        #     for value in goal:
        #         float_goal.append(float(value))
        #     self.goals.append(float_goal)

        # Hardcoded gripper info



    def create_trajectory(self, goal):
        '''Creates a new trajectory with a given goal'''
        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = goal
        point.time_from_start = Duration(sec=4) # Can change trajectory duration here
        traj.points.append(point)
        return traj


    def change_gripper_at_pos(self, goal, new_gripper_pos):
        '''Publish trajectories to move to above goal, move down to goal, move to new gripper position, and move back to above goal'''
        self.commandLock.acquire()

       
        # Publish above pick up position
        above_goal_pos = self.create_trajectory([-1.57,-1.03,-2.08,-1.60,1.57,0.0]) #TODO: get position + certain amount above

        
        print('Publishing above goal position')
        self.publisher_.publish(above_goal_pos)

        time.sleep(4)


        # Publish pick up position
        goal_pos = self.create_trajectory(goal)

        
        print('Publishing goal position')
    
        self.publisher_.publish(goal_pos)

        time.sleep(4)


        # MOVE GRIPPER HERE
        
        print('Moving gripper...')
        
        self.gripper.move_and_wait_for_pos(new_gripper_pos, 255, 255)

        print('Publishing above goal position')
    
        self.publisher_.publish(above_goal_pos)

        time.sleep(4)
        self.commandLock.release()


        #move to neutral position here?


    def pick_up(self, pick_goal):
        '''Pick up from first goal position'''
        self.change_gripper_at_pos(pick_goal, self.gripper_close_pos)
    

    def put_down(self, put_goal):
        '''Put down at second goal position'''
        self.change_gripper_at_pos(put_goal, self.griper_open_pose)


    def pick_up_and_put_down(self,pick_goal,put_goal):
        '''Pick up from first position and put down at second position'''
        self.positions_published = True
        if self.starting_point_ok:
            self.pick_up(pick_goal)
            self.put_down(put_goal)
        
            print('Finished publishing')
        
        elif self.check_starting_point and not self.joint_state_msg_received:
           
            print('Start configuration could not be checked! Check "joint_state" topic!')
        else:
            print("Start configuration is not within configured limits!")

    def transfer(self, pos1, pos2):

            self.pick_up(pos1)
            self.put_down(pos2)
            print('Finished publishing')

    def joint_state_callback(self, msg):
    
        print('Entering Callback')
    
        if not self.joint_state_msg_received:
            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    print(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            if not self.positions_published:
                self.pick_up_and_put_down()
            return


# def main(args=None):
#     rclpy.init(args=args)

#     publisher_joint_trajectory = PublisherJointTrajectory()

#     rclpy.spin(publisher_joint_trajectory)
#     publisher_joint_trajectory.destroy_node()
#     rclpy.shutdown()


if __name__ == "__main__":
    pass
    # main()
