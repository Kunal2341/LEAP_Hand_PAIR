import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Custom class for reading positions
class DynamixelPosReader:
    def __init__(self, port_handler, packet_handler, dxl_id, pos_scale=2.0 * np.pi / 4096):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.dxl_id = dxl_id
        self.pos_scale = pos_scale

    def read_position(self):
        position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        return position

    def get_position_radians(self):
        raw_position = self.read_position()
        return raw_position * self.pos_scale


# Control table address
ADDR_PRO_PRESENT_POSITION = 132
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_TORQUE_ENABLE = 64
# Protocol version
PROTOCOL_VERSION = 2.0
# Default setting
DXL_IDs = list(range(0, 16))

BAUDRATE = 4000000
DEVICENAME = '/dev/ttyUSB2'

POS_SCALE = 2.0 * np.pi / 4096  # Scale factor for converting to radians



TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    input()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    input()
    quit()

torque_setting = TORQUE_DISABLE  # or TORQUE_DISABLE

for dxl_id in DXL_IDs:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, torque_setting)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"ID {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        print(f"Motor ID {dxl_id} torque has been {'enabled' if torque_setting == TORQUE_ENABLE else 'disabled'}")
    
# Initialize PosReader with the Dynamixel ID
#pos_reader = DynamixelPosReader(portHandler, packetHandler, DXL_ID)
positions_radians = []


while True:
    positions_radians.clear()  # Clear the list for fresh readings in each iteration
    
    for dxl_id in DXL_IDs:
        # Read present position for each motor
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("ID %d: %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            print("ID %d: %s" % (dxl_id, packetHandler.getRxPacketError(dxl_error)))
        else:
            # Convert to radians and append to the list
            position_radians = dxl_present_position * POS_SCALE
            positions_radians.append(position_radians)

    # Print the array of positions in radians
    print(f"Positions in Radians: {positions_radians}")

    time.sleep(0.5)  # 500ms pause for readability
