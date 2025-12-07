#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from com760_yourBcode.msg import YourBcodeLeaderS1Command

def broadcast_frames():
    # Initialize the ROS node
    rospy.init_node('formation_control', anonymous=True)

    # TF2 broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Publisher for leader's command
    command_pub = rospy.Publisher('/yourBcodeS1Leader/command', YourBcodeLeaderS1Command, queue_size=10)

    rate = rospy.Rate(10)  # Publish rate

    while not rospy.is_shutdown():
        # Broadcast leader frame
        leader_tf = TransformStamped()
        leader_tf.header.stamp = rospy.Time.now()
        leader_tf.header.frame_id = "world"
        leader_tf.child_frame_id = "leader"
        leader_tf.transform.translation.x = 5.5
        leader_tf.transform.translation.y = 5.5
        leader_tf.transform.rotation.z = 0.0  # No rotation
        tf_broadcaster.sendTransform(leader_tf)

        # Broadcast follower A frame
        followerA_tf = TransformStamped()
        followerA_tf.header.stamp = rospy.Time.now()
        followerA_tf.header.frame_id = "leader"
        followerA_tf.child_frame_id = "followerA"
        followerA_tf.transform.translation.x = -1.0  # Offset to the left
        followerA_tf.transform.translation.y = 0.0
        tf_broadcaster.sendTransform(followerA_tf)

        # Broadcast follower B frame
        followerB_tf = TransformStamped()
        followerB_tf.header.stamp = rospy.Time.now()
        followerB_tf.header.frame_id = "leader"
        followerB_tf.child_frame_id = "followerB"
        followerB_tf.transform.translation.x = 1.0  # Offset to the right
        followerB_tf.transform.translation.y = 0.0
        tf_broadcaster.sendTransform(followerB_tf)

        # Publish commands for followers
        leader_command = YourBcodeLeaderS1Command()
        leader_command.target_x = 5.5  # Leader's x
        leader_command.target_y = 5.5  # Leader's y
        leader_command.orientation = 0.0  # Leader's orientation
        command_pub.publish(leader_command)

        rate.sleep()

if __name__ == "__main__":
    try:
        broadcast_frames()
    except rospy.ROSInterruptException:
        pass
