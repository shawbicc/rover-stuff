#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import numpy as np
import serial

# import sys
# sys.path.insert(0, '..')  # Add the directory containing config.py to the Python path

from config import WHEEL_PORT

# initializing
ser = serial.Serial(WHEEL_PORT, 9600) # SET COM PORT ACCORDING TO ARDUINO

class JoyToTwist:
    def __init__(self):
        # Initialize the node
        rospy.init_node('joy_to_serial_node')
        # Create a subscriber for the /joy topic
        rospy.Subscriber('/joy', Joy, self.joystick_callback)
		# Controller variables
        self.SCALE = 0.5
        self.SCALING_FACTOR = 0.05
        self.prev_joy_data = None

    def joystick_callback(self, data):
        # publish if the joystick data is different than the previous one
        current_joy_data = np.array(data.axes + data.buttons)
        if self.prev_joy_data is None or not np.array_equal(self.prev_joy_data, current_joy_data):

            # increase SCALE if UP BUTTON is pressed
            if data.axes[5] > 0.9 and self.SCALE < 1.0:
                self.SCALE = self.SCALE + self.SCALING_FACTOR

            # reduce SCALE if DOWN BUTTON is pressed
            if data.axes[5] < -0.9 and self.SCALE > 0.0:
                self.SCALE = self.SCALE - self.SCALING_FACTOR

            if self.SCALE > 1.0:
                self.SCALE = 1.0
            if self.SCALE < -1.0:
                self.SCALE = -1.0

            # These indices might need adjustment depending on your joystick

            x = (data.buttons[7] - data.buttons[6]) * self.SCALE * 255 # Scale as necessary
            if x > 1.0*255:
                x = 1.0*255
            if x < -1.0*255:
                x = -1.0*255
            
            z = data.axes[0] * (-self.SCALE) * 255 # Scale as necessary
            if z > 1.0*255:
                z = 1.0*255
            if z < -1.0*255:
                z = -1.0*255

            # Control for base, shoulder, elbow
            b = (data.buttons[4] * -1) + data.buttons[5]
            s = int(data.axes[2])
            e = (data.buttons[2] * -1) + data.buttons[0]


        # Publish the Twist message to the /cmd_vel topic
            ser.write("x {}".format(x))
            ser.write("z {}".format(z))
            ser.write("b {}".format(b))
            ser.write("s {}".format(s))
            ser.write("e {}".format(e))
            rospy.loginfo("Published to {}: x:{}, z:{}, base:{}, shoulder:{}, elbow:{}".format(WHEEL_PORT, x, z, b, s, e))

            # update the previous data
            self.prev_joy_data = current_joy_data

if __name__ == '__main__':
    try:
        joy_to_twist = JoyToTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
