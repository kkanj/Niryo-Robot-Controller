import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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
    learning_mode_pub = rospy.Publisher('/niryo_one/learning_mode', Bool, queue_size=10)
    rate = rospy.Rate(10)

    # Disable learning mode
    learning_mode_pub.publish(Bool(data=False))
    rospy.sleep(1)  # Give some time for the mode to change
    rospy.loginfo("Learning mode disabled")

    print("Use 'WASDQE' to control the robot. Press 'x' to exit.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in MOVE_BINDINGS:
                x, y, z, yaw = MOVE_BINDINGS[key]
                print("Key pressed: {}, Movement: {}, {}, {}, {}".format(key, x, y, z, yaw))
                twist = Twist()
                twist.linear.x = x * 0.1
                twist.linear.y = y * 0.1
                twist.linear.z = z * 0.1
                twist.angular.z = yaw * 0.1
                pub.publish(twist)
                rospy.loginfo("Published twist: {}".format(twist))
            elif key == 'x':  # Exit
                print("Exiting...")
                break
            else:
                # Stop robot if invalid key pressed
                pub.publish(Twist())
                rospy.loginfo("Published stop twist")
            rate.sleep()
    except Exception as e:
        print("An error occurred: {}".format(e))
    finally:
        pub.publish(Twist())  # Stop the robot when exiting
        rospy.loginfo("Published stop twist on exit")

if __name__ == '__main__':
    main()