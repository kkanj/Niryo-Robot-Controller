import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_robot():
    rospy.init_node('test_movement', anonymous=True)
    pub = rospy.Publisher('/niryo_one_follow_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

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