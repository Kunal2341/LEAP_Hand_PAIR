#!/usr/bin/env python3
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

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
from pynput.keyboard import Key, Listener
import threading

class KeyboardController:
    def __init__(self, leap_node):
        self.leap_node = leap_node
        self.ctTorque = 0
    def on_press(self, key):
        homeRad = np.array([3.1492627, 1.6444274, 4.847379,  3.1768742, 
                            4.7522726, 3.1415927, 3.1400588, 4.715457, 
                            3.118583, 3.0771654, 3.103243, 3.0771654, 
                            3.0802333, 3.104777, 3.028078,  3.0633597])
        graspRad = np.array([3.8487577, 2.3331847, 5.608234, 4.0052238, 
                            4.6479616, 1.7932235, 3.034214, 5.74169, 
                            3.6125247, 4.868855, 4.414797, 4.9010687, 
                            4.697049, 3.342544, 3.8686996, 3.351748])
        starPosRad = np.array([3.123185, 2.1997285, 6.688156, 3.6968937, 
                                6.3169327, 2.2120004, 3.1630683, 5.7877097, 
                                3.117049, 3.2198257, 2.9943304, 3.0495539, 
                                3.0771654, 3.181476, 3.2535732, 3.2551072])
        bigGrasp = np.array([3.4468548, 2.5786216, 6.4135737, 4.5006995, 
                            5.108156, 1.59534, 3.113981, 6.0070686, 
                            4.407127, 4.9210105, 3.282719, 4.891865, 
                            4.2660007, 3.0618258, 4.4316707, 3.2198257])
        if key == Key.right:
            # Increase PID values by 10%
            self.leap_node.adjust_pid(1.1)
            print("PID values increased by 10%.")
        elif key == Key.left:
            # Decrease PID values by 10%
            self.leap_node.adjust_pid(0.9)
            print("PID values decreased by 10%.")
        elif hasattr(key, 'char'):  # Check if key event has 'char' attribute
            if key.char == '1':
                print("Moving to home position...")
                self.leap_node.set_leap(homeRad)
            elif key.char == '2':
                print("Moving to grasp position...")
                self.leap_node.set_leap(graspRad)
            elif key.char == '3':
                print("Moving to star position...")
                self.leap_node.set_leap(starPosRad)
            elif key.char == '4':
                print("Moving to big grasp position...")
                self.leap_node.set_leap(bigGrasp)
        elif key == Key.space:
            # Toggle torque enabled
            if self.ctTorque % 2 == 0:
                self.leap_node.set_torque_False()
                print(f"Torque Enabled: False")
                self.ctTorque += 1
            else:
                self.leap_node.set_torque_True()
                print(f"Torque Enabled: True")
                self.ctTorque += 1


    def on_release(self, key):
        if key == Key.esc:
            # Stop listener
            return False

    def start(self):
        # Collect events until released
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current

        #I like 115, 20

        self.kP = 300 #37 #600 
        self.kI = 0
        self.kD =  100 #12 # 200
        self.curr_lim = 350 #Max number for current 
        """
        How the hand is numbered:


        0 - 4 - 8 - 12      (A)     
        1 - 5 - 9 - 13      (B)
        2 - 6 - 10 - 14     (C)
        3 - 7 - 11 - 15     (D)

        """
        
        homeRad = np.array([3.1492627, 1.6444274, 4.847379,  3.1768742, 
                            4.7522726, 3.1415927, 3.1400588, 4.715457, 
                            3.118583, 3.0771654, 3.103243, 3.0771654, 
                            3.0802333, 3.104777, 3.028078,  3.0633597])
        graspRad = np.array([3.8487577, 2.3331847, 5.608234, 4.0052238, 
                            4.6479616, 1.7932235, 3.034214, 5.74169, 
                            3.6125247, 4.868855, 4.414797, 4.9010687, 
                            4.697049, 3.342544, 3.8686996, 3.351748])
        #destination = [3.8487577, 2.3331847, 5.608234, 4.0052238, 4.6479616, 1.7932235, 3.034214, 5.74169, 3.6125247, 4.868855, 4.414797, 4.9010687, 4.697049, 3.342544, 3.8686996, 3.351748]
        #homeRad = np.array(destination)

        homeDeg = homeRad / 0.0015339807878856412 #self.dxl_client._pos_vel_cur_reader.pos_scale
        # homeRadStr = [f"{val:.5f}" for val in homeRad]
        # homeDegStr = [f"{val:.1f}" for val in homeDeg]

        # Function to print 4 values per row
        def print_values(arr, title):
            print(title + " :[")
            for i in range(0, len(arr), 4):
                print("   ", ", ".join(arr[i:i+4]))
            print("]")

        # Print formatted strings
        # print_values(homeRadStr, "Home position Radians (0 - 2pi) is")
        # print_values(homeDegStr, "Home position Ticks (0-4096) is")
        
        self.prev_pos = self.pos = self.curr_pos = homeRad


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
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    def adjust_pid(self, multiplier):
        self.kP = int(self.kP * multiplier)
        self.kI = int(self.kI * multiplier)
        self.kD = int(self.kD * multiplier)
        print(f"PID adjusted: kP={self.kP}, kI={self.kI}, kD={self.kD}")
        # Apply the new PID values to the motors
        self.apply_pid_settings()
    def apply_pid_settings(self):
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
    
    def set_torque_False(self):
        # Toggle the tracked state and apply the change
        self.dxl_client.set_torque_enabled(self.motors, False)
    def set_torque_True(self):
        # Toggle the tracked state and apply the change
        self.dxl_client.set_torque_enabled(self.motors, True)
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

def main(**kwargs):
    leap_hand = LeapNode()
    kb_controller = KeyboardController(leap_hand)
    kb_thread = threading.Thread(target=kb_controller.start)
    kb_thread.start()

    #while kb_thread.is_alive():
        #print("Position: " + str(leap_hand.read_pos()))
        #time.sleep(0.03)



if __name__ == "__main__":
    main()

