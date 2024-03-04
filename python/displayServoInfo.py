import numpy as np
import time
import dynamixel_sdk  # Import the Dynamixel SDK

# Constants for conversion
TICKS_PER_REVOLUTION = 4096
DEGREES_PER_REVOLUTION = 360
RADIANS_PER_REVOLUTION = 2 * np.pi
ADDR_PRESENT_POSITION = 132
# Motor ID to monitor
motor_id_to_monitor = 1  # Example: Motor ID 1

# Setup Dynamixel Client (assuming this is part of the DynamixelClient class)
class DynamixelMonitor:
    def __init__(self, port, baudrate, motor_id):
        self.port = dynamixel_sdk.PortHandler(port)
        self.packet_handler = dynamixel_sdk.PacketHandler(2.0)  # Protocol version 2.0
        self.baudrate = baudrate
        self.motor_id = motor_id
        
        if not self.port.openPort():
            raise IOError("Failed to open port")
        
        if not self.port.setBaudRate(baudrate):
            raise IOError("Failed to set baudrate")
    
    def read_position(self):
        position, result, error = self.packet_handler.read4ByteTxRx(self.port, self.motor_id, ADDR_PRESENT_POSITION)
        if result != dynamixel_sdk.COMM_SUCCESS:
            print("Communication error")
        elif error != 0:
            print("Error occurred")
        return position
    
    def close(self):
        self.port.closePort()

# Conversion functions
def ticks_to_degrees(ticks):
    return ticks * (DEGREES_PER_REVOLUTION / TICKS_PER_REVOLUTION)

def ticks_to_radians(ticks):
    return ticks * (RADIANS_PER_REVOLUTION / TICKS_PER_REVOLUTION)

# Initialize Dynamixel Monitor
monitor = DynamixelMonitor('/dev/ttyUSB3', 4000000, motor_id_to_monitor)

try:
    while True:
        # Read the current position in ticks
        position_ticks = monitor.read_position()
        
        # Convert position to degrees and radians
        position_degrees = ticks_to_degrees(position_ticks)
        position_radians = ticks_to_radians(position_ticks)
        
        # Print the information
        print(f"Motor ID: {motor_id_to_monitor}")
        print(f"Position (ticks): {position_ticks}")
        print(f"Position (degrees): {position_degrees:.2f}")
        print(f"Position (radians): {position_radians:.2f}")
        
        # Add any other metrics you wish to monitor (e.g., velocity, current) in a similar manner
        
        time.sleep(0.3)  # Update rate of 1 second

finally:
    monitor.close()