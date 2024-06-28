import rospy
from getch import getch  # Import getch library for keyboard input
from geometry_msgs.msg import TwistStamped

# Define linear and angular velocity scaling factors
linear_vel_scale = 0.1  # Meters per second (m/s) per key press
angular_vel_scale = 0.5  # Radians per second (rad/s) per key press

# Initialize Twist message
twist_msg = TwistStamped()

def main():
  rospy.init_node('keyboard_teleop')

  # Create a publisher for the Twist message
  pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)

  # Set the publishing rate
  rate = rospy.Rate(10)  # 10 Hz

  # Main loop to handle keyboard input and publish messages
  while not rospy.is_shutdown():
    key = getch()  # Get the pressed key

    # Clear any previously set velocities
    twist_msg.twist.linear.x = 0.0
    twist_msg.twist.linear.y = 0.0
    twist_msg.twist.linear.z = 0.0
    twist_msg.twist.angular.y = 0.0
    twist_msg.twist.angular.z = 0.0

    # Set linear and angular velocities based on pressed key
    if key == 'w':
      twist_msg.twist.linear.x = linear_vel_scale
    elif key == 's':
      twist_msg.twist.linear.x = -linear_vel_scale
    elif key == 'a':
      twist_msg.twist.linear.y = linear_vel_scale
    elif key == 'd':
      twist_msg.twist.linear.y = -linear_vel_scale
    elif key == 'e':
      twist_msg.twist.linear.z = linear_vel_scale
    elif key == 'q':
      twist_msg.twist.linear.z = -linear_vel_scale
    elif key == 'f':
      twist_msg.twist.angular.y = angular_vel_scale
    elif key == 'g':
      twist_msg.twist.angular.y = -angular_vel_scale
    elif key == 'r':
      twist_msg.twist.angular.z = angular_vel_scale
    elif key == 't':
      twist_msg.twist.angular.z = -angular_vel_scale

    # No need to handle a stop key, the robot stops when no key is pressed

    # Set header timestamp automatically
    twist_msg.header.stamp = rospy.Time.now()

    # Publish the Twist message
    pub.publish(twist_msg)
    print(end='\r')
    rate.sleep()

if __name__ == '__main__':
  main()
