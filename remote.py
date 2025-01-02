#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Key bindings for movement
MOVE_BINDINGS = {
    'w': (1, 0, 0, 0),  # Forward
    's': (-1, 0, 0, 0), # Backward
    'a': (0, 1, 0, 0),  # Left
    'd': (0, -1, 0, 0), # Right
    'q': (0, 0, 1, 0),  # Up
    'e': (0, 0, -1, 0), # Down
}

def get_key():
    """Capture key press from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('remote_control')
    pub = rospy.Publisher('/niryo_one/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    print("Use 'WASDQE' to control the robot. Press 'x' to exit.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in MOVE_BINDINGS:
                x, y, z, yaw = MOVE_BINDINGS[key]
                twist = Twist()
                twist.linear.x = x * 0.1
                twist.linear.y = y * 0.1
                twist.linear.z = z * 0.1
                twist.angular.z = yaw * 0.1
                pub.publish(twist)
            elif key == 'x':  # Exit
                print("Exiting...")
                break
            else:
                # Stop robot if invalid key pressed
                pub.publish(Twist())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
