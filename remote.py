import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Keys to move some joints (example: joint 1 and joint 2)
MOVE_BINDINGS = {
    'w': (0.3, 0),   # Increase joint_1 angle
    's': (-0.3, 0),  # Decrease joint_1 angle
    'a': (0, 0.3),   # Increase joint_2 angle
    'd': (0, -0.3)   # Decrease joint_2 angle
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
    rate = rospy.Rate(10)

    # Initial joint positions
    joint_values = [0, 0, 0, 0, 0, 0]

    print("Use 'WASD' to move joints 1 and 2. Press 'x' to exit.")

    while not rospy.is_shutdown():
        key = get_key()
        if key in MOVE_BINDINGS:
            delta_j1, delta_j2 = MOVE_BINDINGS[key]
            joint_values[0] += delta_j1
            joint_values[1] += delta_j2

            # Create trajectory message
            traj = JointTrajectory()
            traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

            point = JointTrajectoryPoint()
            point.positions = joint_values
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