#!/usr/bin/env python3
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
from pynput.keyboard import Key, Listener
import sys
#######################################################
"""
Click 0 1 2 3 for each finger to save their current positions
Click q for all saved values
0 - thumb
1 - right
2 - middle
3 - left


WRONG

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
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, False)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        #self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
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
# Global variables to store positions
positions = [None] * 16  # Assuming 16 positions to cover all indices for keys 0-3.

def on_press(key):
    try:
        if key.char in ['0', '1', '2', '3']:
            index = int(key.char)
            # Calculate the specific indices to update based on the key pressed.
            indices = [index, index + 4, index + 8, index + 12]
            # Fetch the current positions.
            current_positions = leap_hand.read_pos()
            # Update the positions at the calculated indices.
            for i in indices:
                positions[i] = current_positions[i]
            print(f"Saved positions for key {key.char}: {[current_positions[i] for i in indices]}")
        elif key.char == 'q':
            # Ensure all positions have been filled; otherwise, indicate missing values.
            if None in positions:
                print("Warning: Not all positions have been set. Missing values are marked as None.")
            print("Final saved positions:", positions)
            return False  # Stops the listener and exits the loop
    except AttributeError:
        pass  # Handle special keys here if needed



if __name__ == "__main__":
    leap_hand = LeapNode()
    # Start listening to keystrokes
    with Listener(on_press=on_press) as listener:
        listener.join()



#destination = [3.500544, 2.063204, 5.163379, 3.512816, 4.396389, 2.6231072, 3.121651, 5.286098, 4.0558453, 5.026855, 4.7752824, 5.017651, 4.6234183, 3.34101, 3.6401365, 3.5665054]

