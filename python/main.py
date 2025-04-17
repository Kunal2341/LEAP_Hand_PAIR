#!/usr/bin/env python3
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
import sys
#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
#http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

#LEAP hand conventions:
#180 is flat out for the index, middle, ring, fingers, and positive is closing more and more.

"""
########################################################
class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        homeRad = np.array([3.1492627, 1.6444274, 4.847379,  3.1768742, 
                            4.7522726, 3.1415927, 3.1400588, 4.715457, 
                            3.118583, 3.0771654, 3.103243, 3.0771654, 
                            3.0802333, 3.104777, 3.028078,  3.0633597])
        self.prev_pos = self.pos = self.curr_pos = homeRad
        
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB3', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM7', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        # self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    
    def set_from_gen_dex(self, gen_dex):
        gen_dex = np.array(gen_dex).flatten()
        allegro_joints = gen_dex[12:28]
        # mapped_allero_joints = np.zeros(16)
        # joints = np.array(allegro_joints)
        # leap_order = [
        #     0, 1, 2, 3,      # row 0: A0 B0 C0 D0
        #     4, 5, 6, 7,      # row 1: A1 B1 C1 D1
        #     8, 9, 10, 11,    # row 2: A2 B2 C2 D2
        #     12, 13, 14, 15   # row 3: A3 B3 C3 D3
        # ]
        # mapped_allero_joints = joints[leap_order]

        # mapped_allero_joints = np.zeros(16)
        # mapped_allero_joints[0] = allegro_joints[0] #A0
        # mapped_allero_joints[1] = allegro_joints[4] #A1 
        # mapped_allero_joints[2] = allegro_joints[8] #A2
        # mapped_allero_joints[3] = allegro_joints[12] #A3

        # mapped_allero_joints[4] = allegro_joints[1] #B0
        # mapped_allero_joints[5] = allegro_joints[5] #B1
        # mapped_allero_joints[6] = allegro_joints[9] #B2        
        # mapped_allero_joints[7] = allegro_joints[13] #B3

        # mapped_allero_joints[8] = allegro_joints[2] #C0
        # mapped_allero_joints[9] = allegro_joints[6] #C1
        # mapped_allero_joints[10] = allegro_joints[10] #C2
        # mapped_allero_joints[11] = allegro_joints[14] #C3
        
        # mapped_allero_joints[12] = allegro_joints[3] #D0
        # mapped_allero_joints[13] = allegro_joints[7] #D1
        # mapped_allero_joints[14] = allegro_joints[11] #D2
        # mapped_allero_joints[15] = allegro_joints[15] #D3


        self.set_allegro(allegro_joints)

    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
#init the node
def main(**kwargs):
    leap_hand = LeapNode()
    if len(sys.argv) != 2:
        print("Usage: python main.py <path_to_npy_file>")
        sys.exit(1)

    file_path = sys.argv[1]

    try:
        data = np.load(file_path)
        dex_gen = np.zeros(28)
        dex_gen = data.reshape(1, 28)

    except FileNotFoundError:
        print(f"Error: File not found at '{file_path}'")
    except Exception as e:
        print(f"An error occurred: {e}")

    while True:

        homeRad = np.array([3.1492627, 1.6444274, 4.847379,  3.1768742, 
        4.7522726, 3.1415927, 3.1400588, 4.715457, 
        3.118583, 3.0771654, 3.103243, 3.0771654, 
        3.0802333, 3.104777, 3.028078,  3.0633597])

        leap_hand.set_from_gen_dex(dex_gen)
        print("Position: " + str(leap_hand.read_pos()))
        time.sleep(0.03)


if __name__ == "__main__":
    main()
