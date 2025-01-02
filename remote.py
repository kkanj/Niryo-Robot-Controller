import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Movement increment per key press (repeated at ~10 Hz). 
# Feel free to adjust this value if you need slower or faster movement.
MOVE_SPEED = 0.07

# Key bindings for each joint (joint_1 through joint_6)
# Joint 1: w / s
# Joint 2: a / d
# Joint 3: e / q
# Joint 4: r / f
# Joint 5: t / g
# Joint 6: y / h
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

def main():
    rospy.init_node('remote_trajectory_control', anonymous=True)
    pub = rospy.Publisher('/niryo_one_follow_joint_trajectory_controller/command',
                          JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initial joint positions
    joint_values = [0, 0, 0, 0, 0, 0]

    print("Use these keys to move each joint (1-6):")
    print(" Joint 1: w / s")
    print(" Joint 2: a / d")
    print(" Joint 3: e / q")
    print(" Joint 4: r / f")
    print(" Joint 5: t / g")
    print(" Joint 6: y / h")
    print("Press 'x' to exit.")

    while not rospy.is_shutdown():
        key = get_key()
        if key in MOVE_BINDINGS:
            deltas = MOVE_BINDINGS[key]
            for i in range(len(joint_values)):
                joint_values[i] += deltas[i]

            # Create trajectory message
            traj = JointTrajectory()
            traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

            point = JointTrajectoryPoint()
            point.positions = joint_values
            # Increase this duration if you want the robot to move more slowly to each new position
            point.time_from_start = rospy.Duration(1)

            traj.points.append(point)
            pub.publish(traj)
            rospy.loginfo("Published joint trajectory: {}".format(traj))
        elif key == 'x':
            print("Exiting...")
            break
        rate.sleep()

if __name__ == '__main__':
    main()