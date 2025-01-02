import sys
import termios
import tty
import rospy
import actionlib

# Import your Niryo tool action messages (adjust if needed)
from niryo_one_msgs.msg import ToolAction, ToolGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

def toggle_gripper_action(currently_open):
    """
    Toggles the gripper via the /niryo_one/tool_action action server.
    Adjust the 'goal.action' or other fields if your Niryo version differs.
    """
    # Action client for the Niryo One tool action
    client = actionlib.SimpleActionClient('/niryo_one/tool_action', ToolAction)
    rospy.loginfo("Waiting for /niryo_one/tool_action server...")
    client.wait_for_server()

    # Prepare the goal (adjust fields if needed)
    goal = ToolGoal()
    if currently_open:
        rospy.loginfo("Closing gripper via tool action...")
        goal.action = 2  # Example: 2 might mean "close"
    else:
        rospy.loginfo("Opening gripper via tool action...")
        goal.action = 1  # Example: 1 might mean "open"
    
    # Send goal and wait
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Gripper action complete.")

    # Toggle state
    return not currently_open

def main():
    rospy.init_node('remote_trajectory_control', anonymous=True)

    # Joint trajectory publisher (just like before)
    pub = rospy.Publisher('/niryo_one_follow_joint_trajectory_controller/command',
                          JointTrajectory,
                          queue_size=10)
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
    print("Press SPACE to toggle the gripper (via /niryo_one/tool_action).")
    print("Press 'x' to exit.")

    while not rospy.is_shutdown():
        key = get_key()

        # Toggle gripper using SPACE
        if key == ' ':
            # Toggle state using the Action server
            gripper_open = toggle_gripper_action(gripper_open)

        elif key in MOVE_BINDINGS:
            deltas = MOVE_BINDINGS[key]
            for i in range(len(joint_values)):
                joint_values[i] += deltas[i]

            # Create & publish the trajectory
            traj = JointTrajectory()
            traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start = rospy.Duration(1)  # 1 second to reach new positions
            traj.points.append(point)

            pub.publish(traj)
            rospy.loginfo("Published joint trajectory: %s", traj)

        elif key == 'x':
            print("Exiting...")
            break

        rate.sleep()

if __name__ == '__main__':
    main()