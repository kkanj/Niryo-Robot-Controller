#!/usr/bin/env python3

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
rospy.init_node('ping_motor')

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
            rospy.logerr("Failed to ping motor {}: {}".format(motor_id, packet_handler.getTxRxResult(dxl_comm_result)))
            return None
        elif dxl_error != 0:
            rospy.logerr("Error when pinging motor {}: {}".format(motor_id, packet_handler.getRxPacketError(dxl_error)))
            return None
        else:
            rospy.loginfo("Motor {} is responding successfully!".format(motor_id))
            return motor_id
    except Exception as e:
        rospy.logerr("Error getting motor ID: {}".format(e))
        return None

# Get the current motor ID
current_id = get_motor_id(CURRENT_MOTOR_ID)
if current_id is None:
    rospy.logerr("Could not find motor with ID {}".format(CURRENT_MOTOR_ID))
    exit()

# Close the port
port_handler.closePort()