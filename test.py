import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('test_movement', anonymous=True)
    pub = rospy.Publisher('/niryo_one/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Define a simple movement pattern
    movements = [
        (0.1, 0, 0, 0),  # Move forward
        (-0.1, 0, 0, 0), # Move backward
        (0, 0.1, 0, 0),  # Move left
        (0, -0.1, 0, 0), # Move right
        (0, 0, 0.1, 0),  # Move up
        (0, 0, -0.1, 0), # Move down
        (0, 0, 0, 0.1),  # Rotate clockwise
        (0, 0, 0, -0.1)  # Rotate counterclockwise
    ]

    try:
        for movement in movements:
            if rospy.is_shutdown():
                break
            twist = Twist()
            twist.linear.x = movement[0]
            twist.linear.y = movement[1]
            twist.linear.z = movement[2]
            twist.angular.z = movement[3]
            pub.publish(twist)
            rospy.loginfo("Published movement: {}".format(movement))
            rate.sleep()

        # Stop the robot after the movements
        pub.publish(Twist())
        rospy.loginfo("Test completed. Robot stopped.")
        rospy.sleep(1)  # Ensure the stop message is processed
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    move_robot()