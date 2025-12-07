#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

# Global variables for turtle poses
leader_pose = Pose()
follower1_pose = Pose()
follower2_pose = Pose()

# Callback for turtle1 (leader) pose
def leader_pose_callback(msg):
    global leader_pose
    leader_pose = msg

# Calculate velocity to follow leader
def calculate_follower_velocity(follower_pose, offset_x, offset_y):
    velocity = Twist()
    # Target position relative to leader
    target_x = leader_pose.x + offset_x
    target_y = leader_pose.y + offset_y

    # Proportional controller for linear and angular velocities
    distance = math.sqrt((target_x - follower_pose.x)**2 + (target_y - follower_pose.y)**2)
    angle_to_target = math.atan2(target_y - follower_pose.y, target_x - follower_pose.x)

    velocity.linear.x = 1.5 * distance
    velocity.angular.z = 4 * (angle_to_target - follower_pose.theta)

    return velocity

# Callback for turtle2 pose
def follower1_pose_callback(msg):
    global follower1_pose
    follower1_pose = msg

# Callback for turtle3 pose
def follower2_pose_callback(msg):
    global follower2_pose
    follower2_pose = msg

if __name__ == '__main__':
    rospy.init_node('leader_follower')

    # Subscribers
    rospy.Subscriber('/turtle1/pose', Pose, leader_pose_callback)
    rospy.Subscriber('/turtle2/pose', Pose, follower1_pose_callback)
    rospy.Subscriber('/turtle3/pose', Pose, follower2_pose_callback)

    # Publishers
    turtle2_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    turtle3_pub = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Calculate velocities for followers
        follower1_velocity = calculate_follower_velocity(follower1_pose, -1.0, -1.0)  # Offset for turtle2
        follower2_velocity = calculate_follower_velocity(follower2_pose, 1.0, -1.0)   # Offset for turtle3

        # Publish velocities
        turtle2_pub.publish(follower1_velocity)
        turtle3_pub.publish(follower2_velocity)

        rate.sleep()
