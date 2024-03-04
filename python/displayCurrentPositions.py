#!/usr/bin/env python3
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

#######################################################
"""
Basically displays the current live stats for the motors
in both ticks and radians

"""
########################################################
class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350 #Max number for current 
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
      
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
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
leap_hand = LeapNode()

# Initialize plotting
fig, ax = plt.subplots()
x_data, y_data = [], [[] for _ in range(len(leap_hand.motors))]  # Prepare y_data to hold lists for each motor
lines = []
texts = []
# Manually defined strong base colors for each group

# Manually define vibrant colors for gradients
color_gradients = ['#FF0000', '#FF4500', '#FF8C00', '#FFD700',  # Red to Yellow Gradient
                   '#1E90FF', '#00BFFF', '#87CEFA', '#B0E0E6',  # Blue to Light Blue Gradient
                   '#32CD32', '#98FB98', '#00FA9A', '#F5FFFA',  # Green to Mint Gradient
                   '#9400D3', '#9932CC', '#BA55D3', '#DDA0DD']  # Dark Violet to Lavender Gradient

# Define labels for each line
labels = ["A1", "B1", "C1", "D1", "A2", "B2", "C2", "D2",
          "A3", "B3", "C3", "D3", "A4", "B4", "C4", "D4"]

for i in range(16):
    line, = ax.plot([], [], color=color_gradients[i], linestyle='-', animated=True)
    lines.append(line)
    text = ax.text(0, 0, labels[i], color='black', fontsize=8, verticalalignment='center')
    texts.append(text)

def init():
    ax.set_xlim(0, 10)  # Initial x-axis limit, will adjust dynamically
    ax.set_ylim(0, 2*np.pi)  # Set y-axis limits to show 0 to 2*pi radians
    ax.set_xlabel('Time')
    ax.set_ylabel('Position (radians) / Scale Factor (0-4096)')
    ax.set_title('Position for Leap Hand')
    return lines + texts

def update(frame):
    x_data.append(frame)  # Assume each frame corresponds to a new time point
    positions = leap_hand.read_pos()  # Read new positions
    # Adjust positions to be within 0 to 2*pi range
    positions = np.mod(positions, 2*np.pi)

    for i, pos in enumerate(positions):
        y_data[i].append(pos)

    for i, (line, text) in enumerate(zip(lines, texts)):
        line.set_data(x_data, y_data[i])
        if len(x_data) > 0 and len(y_data[i]) > 0:
            text.set_position((x_data[-1] + 0.1, y_data[i][-1]))

    if frame >= x_data[0] + 4.5:  # Keep the last 5 seconds in view
        ax.set_xlim(frame - 4.5, frame + 0.5)

    all_positions = np.hstack(y_data)
    y_min, y_max = all_positions.min(), all_positions.max()
    padding = (y_max - y_min) * 0.1
    ax.set_ylim(y_min - padding, y_max + padding)

    # Add secondary y-axis for scale factor 0-4096
    def rad_to_scale(y):
        return y / 0.0015339807878856412
    def scale_to_rad(y):
        return y * 0.0015339807878856412
    secax = ax.secondary_yaxis('right', functions=(rad_to_scale, scale_to_rad))
    secax.set_ylabel('Scale Factor (0-4096)')

    return lines + texts

def on_close(event):
    # Function to execute when the window is closed
    max_positions = [max(pos) for pos in y_data]
    min_positions = [min(pos) for pos in y_data]
    print("Max Positions:", max_positions)
    print("Min Positions:", min_positions)

fig.canvas.mpl_connect('close_event', on_close)

# Run animation
ani = FuncAnimation(fig, update, frames=np.linspace(0, 100, 10000), init_func=init, blit=True, interval=50)

plt.show()

"""
How the hand is numbered:


0 - 4 - 8 - 12      (A)
1 - 5 - 9 - 13      (B)
2 - 6 - 10 - 14     (C)
3 - 7 - 11 - 15     (D)

"""