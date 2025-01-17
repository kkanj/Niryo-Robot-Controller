#!/usr/bin/env python

import rospy
from dynamixel_sdk import *  # Import Dynamixel SDK library

# Setup
PORT_NAME = "/dev/ttyUSB0"  # Update this to your serial port if needed
BAUDRATE = 57600
CURRENT_MOTOR_ID = 5  # The current motor ID (set this to the motor's current ID)
NEW_MOTOR_ID = 6  # The new ID you want to assign to the motor

# Initialize PortHandler and PacketHandler
port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(1.0)

# Initialize ROS node
rospy.init_node('change_motor_id')

# Open the port
if not port_handler.openPort():
    rospy.logerr("Failed to open the port!")
    exit()

# Set the baudrate
if not port_handler.setBaudRate(BAUDRATE):
    rospy.logerr("Failed to set baud rate!")
    exit()

# Function to check the current motor ID
def get_motor_id(motor_id):
    try:
        # Ping the motor to get its current ID
        dxl_comm_result, dxl_error = packet_handler.ping(port_handler, motor_id)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(f"Failed to ping motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
            return None
        elif dxl_error != 0:
            rospy.logerr(f"Error when pinging motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
            return None
        else:
            rospy.loginfo(f"Motor {motor_id} is responding successfully!")
            return motor_id
    except Exception as e:
        rospy.logerr(f"Error getting motor ID: {e}")
        return None

# Get the current motor ID
current_id = get_motor_id(CURRENT_MOTOR_ID)
if current_id is None:
    rospy.logerr(f"Could not find motor with ID {CURRENT_MOTOR_ID}")
    exit()

# Change the motor ID to a new one
def change_motor_id(old_motor_id, new_motor_id):
    try:
        # Change the motor ID by writing to the ID register (address 0x03)
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, old_motor_id, 0x03, new_motor_id)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(f"Failed to change motor ID: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            rospy.logerr(f"Error while changing motor ID: {packet_handler.getRxPacketError(dxl_error)}")
        else:
            rospy.loginfo(f"Motor ID changed successfully to {new_motor_id}")
            return True
    except Exception as e:
        rospy.logerr(f"Error changing motor ID: {e}")
        return False

# Change the motor ID to the new one
if change_motor_id(CURRENT_MOTOR_ID, NEW_MOTOR_ID):
    # Verify the change by pinging the motor with the new ID
    new_motor_id_check = get_motor_id(NEW_MOTOR_ID)
    if new_motor_id_check:
        rospy.loginfo(f"Motor ID has been successfully updated to {NEW_MOTOR_ID}")
    else:
        rospy.logerr("Failed to verify the new motor ID.")

# Close the port
port_handler.closePort()
