#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np


class JoyToTwist:
    def __init__(self):
        # Initialize the node
        rospy.init_node('joy_to_rosserial_node')

        # Create a subscriber for the /joy topic
        rospy.Subscriber('/joy', Joy, self.joystick_callback)
        self.output_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		# Controller variables
        self.SCALE = 1
        self.SCALING_FACTOR = 0
        self.prev_joy_data = None

    def joystick_callback(self, data):
        # publish if the joystick data is different than the previous one
        
        # current_joy_data = np.array(data.axes[0] + data.buttons[4] + data.buttons[5])
        current_joy_data = np.array([round(data.axes[0]), data.buttons[7], data.buttons[6], data.buttons[8], data.buttons[9]])
        if self.prev_joy_data is None or not np.array_equal(self.prev_joy_data, current_joy_data):

            # increase SCALE if UP BUTTON is pressed
            if data.buttons[9] > 0.9 and self.SCALE < 1.0:
                self.SCALE = self.SCALE + self.SCALING_FACTOR

            # reduce SCALE if DOWN BUTTON is pressed
            if data.buttons[8] > 0.9 and self.SCALE > 0.0:
                self.SCALE = self.SCALE - self.SCALING_FACTOR

            if self.SCALE > 1.0:
                self.SCALE = 1.0
            if self.SCALE < -1.0:
                self.SCALE = -1.0

            # These indices might need adjustment depending on your joystick
            output = Twist()
        
            output.linear.x = (data.buttons[7] - data.buttons[6]) * self.SCALE  # Scale as necessary
            if output.linear.x > 1.0:
                output.linear.x = 1.0
            if output.linear.x < -1.0:
                output.linear.x = -1.0
            
            output.angular.z = data.axes[0] * (-self.SCALE)  # Scale as necessary
            if output.angular.z > 1.0:
                output.angular.z = 1.0
            if output.angular.z < -1.0:
                output.angular.z = -1.0


            # Publish the Twist message to the /cmd_vel topic
            self.output_publisher.publish(output)

            rospy.loginfo("Published: x:{}, z:{}".format(output.linear.x, output.angular.z))

            # update the previous data
        self.prev_joy_data = current_joy_data

if __name__ == '__main__':
    try:
        joy_to_twist = JoyToTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

