import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool

def move_robot():
    rospy.init_node('test_movement', anonymous=True)
    pub = rospy.Publisher('/niryo_one_follow_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    learning_mode_pub = rospy.Publisher('/niryo_one/learning_mode', Bool, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Disable learning mode
    rospy.loginfo("Disabling learning mode...")
    learning_mode_pub.publish(Bool(data=False))
    rospy.sleep(2)  # Give some time for the mode to change

    # Verify learning mode is off
    learning_mode_status = rospy.wait_for_message('/niryo_one/learning_mode', Bool)
    if learning_mode_status.data:
        rospy.logwarn("Failed to disable learning mode. Please disable it manually.")
        print(learning_mode_status)
        return

    # Define a simple joint trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    point = JointTrajectoryPoint()
    point.positions = [0.5, 0, 0, 0, 0, 0]  # Example positions for each joint
    point.time_from_start = rospy.Duration(2)  # Move to the position in 2 seconds

    trajectory.points.append(point)

    try:
        while not rospy.is_shutdown():
            pub.publish(trajectory)
            rospy.loginfo("Published trajectory: {}".format(trajectory))
            rate.sleep()
            break  # Publish once and exit

        rospy.loginfo("Test completed.")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    move_robot()