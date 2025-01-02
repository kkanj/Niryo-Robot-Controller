import sys
import termios
import tty
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# Example gripper service messages (adjust to match your setup!)
from niryo_one_msgs.srv import SetInt, SetIntRequest

MOVE_SPEED = 0.01

# Key bindings for each joint (1-6)
MOVE_BINDINGS = {
    'w': ( MOVE_SPEED,      0,          0,          0,          0,          0 ),
    's': (-MOVE_SPEED,      0,          0,          0,          0,          0 ),
    'a': (0,                MOVE_SPEED, 0,          0,          0,          0 ),
    'd': (0,               -MOVE_SPEED, 0,          0,          0,          0 ),
    'e': (0,                0,         MOVE_SPEED,  0,          0,          0 ),
    'q': (0,                0,        -MOVE_SPEED,  0,          0,          0 ),
    'r': (0,                0,          0,         MOVE_SPEED,  0,          0 ),
    'f': (0,                0,          0,        -MOVE_SPEED,  0,          0 ),
    't': (0,                0,          0,          0,         MOVE_SPEED,  0 ),
    'g': (0,                0,          0,          0,        -MOVE_SPEED,  0 ),
    'y': (0,                0,          0,          0,          0,         MOVE_SPEED),
    'h': (0,                0,          0,          0,          0,        -MOVE_SPEED)
}

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def toggle_gripper(currently_open):
    """
    Toggles between open_gripper and close_gripper services.
    Adjust service names and request types to match your actual setup.
    """
    if currently_open:
        rospy.loginfo("Closing gripper...")
        rospy.wait_for_service('/niryo_one/close_gripper')
        close_srv = rospy.ServiceProxy('/niryo_one/close_gripper', SetInt)
        close_srv(SetIntRequest(value=1))  # Adjust request as needed
        return False
    else:
        rospy.loginfo("Opening gripper...")
        rospy.wait_for_service('/niryo_one/open_gripper')
        open_srv = rospy.ServiceProxy('/niryo_one/open_gripper', SetInt)
        open_srv(SetIntRequest(value=1))   # Adjust request as needed
        return True

def main():
    rospy.init_node('remote_trajectory_control', anonymous=True)
    pub = rospy.Publisher('/niryo_one_follow_joint_trajectory_controller/command',
                          JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)

    # Initial joint positions
    joint_values = [0, 0, 0, 0, 0, 0]
    gripper_open = False

    print("Use these keys to move each joint (1-6):")
    print(" Joint 1: w / s")
    print(" Joint 2: a / d")
    print(" Joint 3: e / q")
    print(" Joint 4: r / f")
    print(" Joint 5: t / g")
    print(" Joint 6: y / h")
    print("Press SPACE to toggle the gripper.")
    print("Press 'x' to exit.")

    while not rospy.is_shutdown():
        key = get_key()

        # Toggle gripper using SPACE
        if key == ' ':
            gripper_open = toggle_gripper(gripper_open)

        elif key in MOVE_BINDINGS:
            deltas = MOVE_BINDINGS[key]
            for i in range(len(joint_values)):
                joint_values[i] += deltas[i]

            # Build trajectory message
            traj = JointTrajectory()
            traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start = rospy.Duration(1)  # Slower movement with a 1s duration
            traj.points.append(point)

            pub.publish(traj)
            rospy.loginfo("Published joint trajectory: {}".format(traj))

        elif key == 'x':
            print("Exiting...")
            break

        rate.sleep()

if __name__ == '__main__':
    main()