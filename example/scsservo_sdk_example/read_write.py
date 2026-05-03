#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS/SCS), and an URT
# Be sure that SCServo(STS/SMS/SCS) properties are already set as %% ID : 1 / Baudnum : 6 (Baudrate : 1000000)
#

import os
import numpy as np

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *                    # Uses SCServo SDK library

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56
ADDR_SCS_MOVING_STATUS = 66

# Default setting
SCS_ID_0                      = 1                 # SCServo ID : 1
SCS_ID_1                      = 2                 # SCServo ID : 2
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = 'COM3'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = -3000         # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 3000        # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 20          # SCServo moving status threshold
SCS_MOVING_SPEED            = 2000           # SCServo moving speed
SCS_MOVING_ACC              = 0           # SCServo moving acc
protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position
for i in range(len(scs_goal_position)):
    goal_position=scs_goal_position[i]
    if goal_position>0:
        val=0b0000000000000000|abs(goal_position)
        scs_goal_position[i]=val
    elif goal_position<0:
        val=0b1000000000000000|abs(goal_position)
        scs_goal_position[i]=val

print(scs_goal_position)
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Write SCServo acc
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
print('SCServo acc set!')
# Write SCServo speed
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
print('SCServo speed set!')
#Set Zeros
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_TORQUE_ENABLE, 128)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_TORQUE_ENABLE, 128)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
print('SCServo zero set!')

turn_num_0=0
turn_num_1=0

#pre-read
scs_present_position_speed_0, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_PRESENT_POSITION)
if scs_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print(packetHandler.getRxPacketError(scs_error))

scs_present_position_speed_1, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_PRESENT_POSITION)
if scs_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print(packetHandler.getRxPacketError(scs_error))
raw_angle_current_0 = SCS_LOWORD(scs_present_position_speed_0)
raw_angle_current_1 = SCS_LOWORD(scs_present_position_speed_1)

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):#which is the character 'Esc'
        break

    # Write SCServo goal position
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_GOAL_POSITION, scs_goal_position[index])
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_GOAL_POSITION, scs_goal_position[index])
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    i=0
    while i<500:
        # Read SCServo present position 
        scs_present_position_speed_0, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        scs_present_position_speed_1, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        # Read SCServo present status
        scs_present_status_0, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_MOVING_STATUS)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        scs_present_status_1, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_MOVING_STATUS)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        scs_present_position_0 = SCS_LOWORD(scs_present_position_speed_0)
        if abs(scs_present_position_0-raw_angle_current_0)>3500:
            turn_num_0=turn_num_0+int(np.sign(raw_angle_current_0-scs_present_position_0))
        raw_angle_current_0=scs_present_position_0
        angle_current_0=scs_present_position_0+4096*turn_num_0

        scs_present_position_1 = SCS_LOWORD(scs_present_position_speed_1)
        if abs(scs_present_position_1-raw_angle_current_1)>3500:
            turn_num_1=turn_num_1+int(np.sign(raw_angle_current_1-scs_present_position_1))
        raw_angle_current_1=scs_present_position_1
        angle_current_1=scs_present_position_0+4096*turn_num_1

        scs_present_status_0=scs_present_status_0&0x0001
        scs_present_status_1=scs_present_status_1&0x0001

        print(scs_present_status_0, raw_angle_current_0, angle_current_0)
        print(scs_present_status_1, raw_angle_current_1, angle_current_1)
        i=i+1
        # if (abs(scs_goal_position[index] - scs_present_position_speed_0) < SCS_MOVING_STATUS_THRESHOLD) and (abs(scs_goal_position[index] - scs_present_position_speed_1) < SCS_MOVING_STATUS_THRESHOLD):
        #     break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0    

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID_0, ADDR_SCS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID_1, ADDR_SCS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
# Close port
portHandler.closePort()