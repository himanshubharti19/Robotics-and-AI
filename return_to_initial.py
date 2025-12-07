import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Define initial positions of the turtles
initial_positions = {
    "turtle1": {"x": 5.5, "y": 5.5, "theta": 0.0},
    "turtle2": {"x": 4.6, "y": 4.6, "theta": 0.0},
    "turtle3": {"x": 4.6, "y": 6.6, "theta": 0.0},
}

# Current positions of the turtles
current_positions = {}

# Callback to update current position
def pose_callback(data, turtle_name):
    current_positions[turtle_name] = {"x": data.x, "y": data.y, "theta": data.theta}

# Move the turtle to the target position
def move_to_position(turtle_name):
    rospy.init_node(f'{turtle_name}_controller', anonymous=True)
    vel_pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber(f'/{turtle_name}/pose', Pose, pose_callback, turtle_name)

    rate = rospy.Rate(10)  # 10 Hz
    vel_msg = Twist()

    target = initial_positions[turtle_name]
    tolerance = 0.1  # Position tolerance

    while not rospy.is_shutdown():
        if turtle_name not in current_positions:
            continue  # Wait for position data

        # Get current and target positions
        current = current_positions[turtle_name]
        dx = target["x"] - current["x"]
        dy = target["y"] - current["y"]
        distance = math.sqrt(dx**2 + dy**2)

        # Stop if the turtle is close enough to the target
        if distance < tolerance:
            break

        # Calculate linear and angular velocities
        angle_to_target = math.atan2(dy, dx)
        angular_error = angle_to_target - current["theta"]

        vel_msg.linear.x = 1.5 * distance  # Scale factor for smooth motion
        vel_msg.angular.z = 4.0 * angular_error  # Adjust angle quickly

        # Publish the velocity message
        vel_pub.publish(vel_msg)
        rate.sleep()

    # Stop the turtle after reaching the target
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)

if __name__ == "__main__":
    try:
        move_to_position("turtle1")
        move_to_position("turtle2")
        move_to_position("turtle3")
    except rospy.ROSInterruptException:
        pass
