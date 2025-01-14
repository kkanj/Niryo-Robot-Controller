import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from niryo_one_msgs.msg import ToolAction, ToolGoal
from inputs import get_gamepad


def move_robot():

    rospy.init_node('gamepad_trajectory_control', anonymous=True)
    pub = rospy.Publisher('/niryo_one_follow_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)

    MOVE_SPEED = 0.07
    joint_values = [0, 0, 0, 0, 0, 0]
    gripper_open = False

    def toggle_gripper_action(is_open):
        client = actionlib.SimpleActionClient('/niryo_one/tool_action', ToolAction)
        client.wait_for_server()
        goal = ToolGoal()
        goal.action = 1 if not is_open else 2
        client.send_goal(goal)
        client.wait_for_result()
        return not is_open

    while not rospy.is_shutdown():
        events = get_gamepad()
        for event in events:
            if event.code == 'BTN_SOUTH' and event.state == 1:
                gripper_open = toggle_gripper_action(gripper_open)

            elif event.code == 'ABS_Y':
                if event.state < 128:
                    joint_values[0] += MOVE_SPEED
                elif event.state > 128:
                    joint_values[0] -= MOVE_SPEED

            elif event.code == 'ABS_X':
                if event.state < 128:
                    joint_values[1] += MOVE_SPEED
                elif event.state > 128:
                    joint_values[1] -= MOVE_SPEED

            # Add more event checks for other joints or buttons here

            traj = JointTrajectory()
            traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start = rospy.Duration(1)
            traj.points.append(point)
            pub.publish(traj)

        rate.sleep()

    

if __name__ == '__main__':
    move_robot()